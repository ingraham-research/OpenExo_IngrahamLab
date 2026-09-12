#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

#include "ExoBLE.h"
#include "Utilities.h"
#include "Time_Helper.h"
#include "ComsLed.h"
#include "Config.h"
#include "error_codes.h"
#include "Logger.h"
#include "GetBulkChar.h"
#include "SystemReset.h"

#define EXOBLE_DEBUG 0

ExoBLE* ExoBLE::_instance = nullptr;

//BLE link liveness. In the observed failure the Nano keeps running - loop() alive, RT data still
//arriving over I2C, LED still blinking - but nothing reaches the host, and because the disconnect
//event is never processed BLE.connected() stays non-zero, so ExoBLE never re-advertises and the GUI
//cannot find the device again. Nothing on our side looks wrong: sendAclPkt() still returns 0 and
//writeValue() still succeeds. The only trustworthy signal is end-to-end: has the GUI reached us
//lately. Hence the ping. See EXO_BLE_STALL_MS / exo_ble_stall_reset() in SystemReset.h.
static uint32_t s_last_rx_ms = 0;

//Only enforce the timeout once a ping has actually been seen, so connecting with a GUI (or any other
//client) that does not ping can never reboot the board. Fail safe, not fail fast.
static bool s_ping_seen = false;

//What the send path looked like since the last byte arrived from the GUI. Reset on every RX, so
//these describe exactly the silent window the stall detector fires on. See exo_ble_stall_reset().
static uint32_t s_max_write_us    = 0;
static uint32_t s_sends_since_rx  = 0;
static bool     s_write_failed    = false;

//The boot-time reset banner, cached so the connect path can append the connection parameters to it
//without recomputing it. See the ErrorChar write in begin() for why recomputing would be a bug.
static String s_boot_banner;

//Coarse log2 buckets - we get 8 bits total through GPREGRET2, so precision is not the point;
//distinguishing "instant" from "blocking" is.
static uint8_t exo_us_bucket(uint32_t us)
{
    uint32_t v = us >> 6;   //64 us units
    uint8_t b = 0;
    while (v && b < 15) { v >>= 1; b++; }
    return b;
}

//Coarse send-count bucket: 0 = none at all, 1 = 1-9, 2 = 10-99, 3 = 100+. Two bits is enough - the
//distinction that matters is "we were talking into the void" versus "we had nothing to send".
static uint8_t exo_count_bucket(uint32_t c)
{
    if (c == 0)   { return 0; }
    if (c < 10)   { return 1; }
    if (c < 100)  { return 2; }
    return 3;
}

//Layout, 8 bits, packed into GPREGRET2 by exo_ble_stall_reset():
//  bits 0-3  max writeValue() duration bucket, ~2^(w-1) * 64 us (0 = under 64 us)
//  bit  4    a writeValue() returned false at some point
//  bits 5-6  notifications attempted during the silence (see exo_count_bucket)
//  bit  7    ALWAYS 1
//Bit 7 exists so the byte can never be 0. GPREGRET2 is shared with the watchdog stage breadcrumb,
//and a zero there already means "nothing recorded" - without this marker a diag of all-zeros would
//be indistinguishable from a blank, which is exactly the ambiguity that wasted three bench runs.
#define EXO_DIAG_MARKER 0x80u

uint8_t exo_ble_link_diag()
{
    return (uint8_t)((exo_us_bucket(s_max_write_us) & 0x0Fu)
                     | (s_write_failed ? 0x10u : 0x00u)
                     | ((exo_count_bucket(s_sends_since_rx) & 0x03u) << 5)
                     | EXO_DIAG_MARKER);
}

namespace
{
    // 19 bytes is deliberate: the default ATT MTU is 23, leaving 20 usable. Do NOT raise this
    // without first confirming a larger MTU was actually negotiated - writing more than the
    // negotiated MTU silently truncates EVERY notification.
    constexpr size_t kHandshakeChunkSize = 19;

    constexpr unsigned long kInterChunkDelayMs = 20;

    // DO NOT add a writeValue() return check here expecting it to catch dropped notifications.
    // It cannot, and this was tried: ArduinoBLE's BLELocalCharacteristic::writeValue() returns
    // ATT.handleNotify(), which calls HCI.sendAclPkt() and *discards its result*, then returns
    // success whenever any peer is connected. So the return value is non-zero even when the
    // notification never lands. (HCI.sendAclPkt itself blocks on `_pendingPkt >= _maxPkt`, so
    // the Nano's TX path applies backpressure and is not where bytes go missing.)
    // Retrying on that return value is dead code; pumping BLE.poll() inside this loop is worse
    // than dead, because it lets other BLE events dispatch mid-payload and interleave bytes
    // into the stream. Keep this loop dumb.
    bool send_chunked(BLECharacteristic &characteristic, const char *data, size_t len)
    {
        size_t offset = 0;
        while (offset < len)
        {
            const size_t chunk_len = ((len - offset) > kHandshakeChunkSize)
                                         ? kHandshakeChunkSize
                                         : (len - offset);
            characteristic.writeValue((const uint8_t *)(data + offset), (int)chunk_len);
            delay(kInterChunkDelayMs);
            offset += chunk_len;
        }
        return true;
    }

    bool send_handshake_payload(BLECharacteristic &characteristic)
    {
        const char ready_msg[] = "READY";
        if (!send_chunked(characteristic, ready_msg, sizeof(ready_msg) - 1))
        {
            #ifdef SIMPLE_DEBUG
            Serial.print("\nExoBLE::handshake_payload FAILED to send READY");
            #endif
            return false;
        }
        delay(kInterChunkDelayMs);

        const char *raw_payload = rxBuffer_bulkStr;
        if (raw_payload == nullptr || raw_payload[0] == '\0')
        {
            return false;
        }

        static char sanitized_payload[MAX_MESSAGE_SIZE + 2] = {0};
        size_t write_index = 0;
        bool truncated = false;
        sanitized_payload[0] = '\0';

        size_t read_index = 0;
        for (; raw_payload[read_index] != '\0' && read_index < MAX_MESSAGE_SIZE; ++read_index)
        {
            if (write_index >= (MAX_MESSAGE_SIZE - 1))
            {
                truncated = true;
                break;
            }

            char c = raw_payload[read_index];
            if (c == '\r')
            {
                continue;
            }

            sanitized_payload[write_index++] = (c == '\n') ? '|' : c;
        }

        // Overrunning the buffer used to be a bare break - no error, no flag. Say so instead.
        if (raw_payload[read_index] != '\0')
        {
            truncated = true;
        }

        if (write_index >= MAX_MESSAGE_SIZE)
        {
            write_index = MAX_MESSAGE_SIZE - 1;
        }

        sanitized_payload[write_index++] = '\n';
        sanitized_payload[write_index] = '\0';

        // Count rows before sending. '|' separates rows, so the row total is its count.
        size_t row_count = 0;
        size_t value_row_count = 0;
        bool row_start = true;
        for (size_t i = 0; i < write_index; ++i)
        {
            const char c = sanitized_payload[i];
            if (row_start)
            {
                if (c == 'v')
                {
                    value_row_count++;
                }
                row_start = false;
            }
            if (c == '|')
            {
                row_count++;
                row_start = true;
            }
        }

        // Row-count header, sent first: "n,<total rows including this one>|". The GUI compares
        // it against the number of rows it actually parsed, which detects a dropped chunk
        // exactly instead of inferring it from left/right symmetry.
        char header[24];
        const int header_len = snprintf(header, sizeof(header), "n,%u|", (unsigned)(row_count + 1));
        const bool header_ok = (header_len > 0) && ((size_t)header_len < sizeof(header)) &&
                               send_chunked(characteristic, header, (size_t)header_len);
        const bool body_ok = header_ok &&
                             send_chunked(characteristic, sanitized_payload, write_index);
        const size_t chunk_count = (write_index + kHandshakeChunkSize - 1) / kHandshakeChunkSize;

        #ifdef SIMPLE_DEBUG
        Serial.print("\nExoBLE::handshake_payload bytes=");
        Serial.print(write_index);
        Serial.print(" chunks=");
        Serial.print(chunk_count);
        Serial.print(" rows=");
        Serial.print(row_count);
        Serial.print(" value_rows=");
        Serial.print(value_row_count);
        Serial.print(truncated ? " TRUNCATED(buffer full)" : "");
        Serial.print(body_ok ? " sent=OK" : " sent=FAILED(chunk undelivered)");
        #endif

        return body_ok;
    }
}

ExoBLE::ExoBLE()
{
    _instance = this;
}

bool ExoBLE::setup()
{
    //A watchdog surviving a warm reset is still counting while this runs (the nRF52840 WDT is only
    //cleared by a power-on reset or by firing). BLE.begin() is the longest single block on the boot
    //path after the UART waits, so bracket it. Harmless when no watchdog is running.
    exo_wdt_feed();
    exo_wdt_stage(EXO_STAGE_BLE_BEGIN);

    if (!BLE.begin())
    {
        utils::spin_on_error_with("BLE.begin() failed");
        return false;
    }

    //Setup name and initialize data
    String name = utils::remove_all_chars(BLE.address(), ':');
    name.remove(name.length() - MAC_ADDRESS_NAME_LENGTH);
    name = NAME_PREAMBLE + name;

    //Using exo_info namespace defined in Config.h
    String FirmwareVersion = exo_info::FirmwareVersion; //String to add to firmware char
    String PCBVersion = exo_info::PCBVersion;           //String to add to pcb char
    String DeviceName = exo_info::DeviceName;           //String to add to device char

    //Check if the name is null, if it is use the name above, if not check for preamble
    if (DeviceName == "NULL")
    {
        DeviceName = name;
    }
    else
    {
        //Check if the name has the preamble, if not add it
        if (!DeviceName.startsWith(NAME_PREAMBLE))
        {
            DeviceName = NAME_PREAMBLE + DeviceName;
        }
    }

    //Initialize char arrays
    char name_char[name.length()];
    char firmware_char[FirmwareVersion.length()];
    char pcb_char[PCBVersion.length()];
    char device_char[DeviceName.length()];

    //Add data to array
    name.toCharArray(name_char, name.length() + 1);
    FirmwareVersion.toCharArray(firmware_char, FirmwareVersion.length() + 1);
    PCBVersion.toCharArray(pcb_char, PCBVersion.length() + 1);
    DeviceName.toCharArray(device_char, DeviceName.length() + 1);

    //Create pointer that pointes to array
    const char *k_name_pointer = name_char;
    const char *firmware_pointer = firmware_char;
    const char *pcb_pointer = pcb_char;
    const char *device_pointer = device_char;

    //Set name for device
    BLE.setLocalName(k_name_pointer);
    BLE.setDeviceName(k_name_pointer);

    //Initialize GATT DB
    _gatt_db.FirmwareChar.writeValue(firmware_char);
    _gatt_db.PCBChar.writeValue(pcb_char);
    _gatt_db.DeviceChar.writeValue(device_char);
    send_error(0, 0);

    //Park the last reset reason in ErrorChar so the GUI can READ it once, right after connecting.
    //
    //Read rather than notify on purpose: nobody is subscribed at boot, and the notify path would
    //have to fire during the connect handshake, which is exactly the burst that already loses
    //controller rows. A stored value costs the GUI one idle round trip before notifications start.
    //
    //Reusing ErrorChar rather than adding a characteristic is also deliberate: Windows caches the
    //GATT table per device, so a new characteristic can be served stale from that cache and read
    //back as missing. Nothing about the table changes here.
    //
    //send_error() above is a no-op at this point (it early-returns while _connected is 0), so this
    //write is what ErrorChar actually holds until the first runtime error overwrites it.
    {
        //Append what the Teensy saw on the RT link, so one reconnect tells both halves of the
        //story: what the Nano did, and whether I2C was healthy while it happened.
        //Cached, not just written: handle_updates() rewrites ErrorChar at connect to append the
        //connection parameters (see below), and it must NOT re-enter exo_reset_reason_string() to
        //do it. That function latches state out of GPREGRET as a side effect, so calling it twice
        //is how the crash/stall record got erased once before. Cache the string, append to the copy.
        s_boot_banner = exo_reset_reason_string() + exo_link_stats_string();
        char reset_char[s_boot_banner.length() + 1];
        s_boot_banner.toCharArray(reset_char, s_boot_banner.length() + 1);
        _gatt_db.ErrorChar.writeValue(reset_char);
    }

    //Configure services and advertising data
    BLE.setAdvertisedService(_gatt_db.UARTService);

    //UART Chars
    _gatt_db.UARTService.addCharacteristic(_gatt_db.TXChar);
    _gatt_db.UARTService.addCharacteristic(_gatt_db.RXChar);

    //Device Info Chars
    _gatt_db.UARTServiceDeviceInfo.addCharacteristic(_gatt_db.PCBChar);
    _gatt_db.UARTServiceDeviceInfo.addCharacteristic(_gatt_db.FirmwareChar);
    _gatt_db.UARTServiceDeviceInfo.addCharacteristic(_gatt_db.DeviceChar);

    //Error Char
    _gatt_db.ErrorService.addCharacteristic(_gatt_db.ErrorChar);

    BLE.addService(_gatt_db.UARTService);
    BLE.addService(_gatt_db.UARTServiceDeviceInfo);
    BLE.addService(_gatt_db.ErrorService);

    _gatt_db.RXChar.setEventHandler(BLEWritten, ble_rx::on_rx_recieved);


    // When the central subscribes to notifications on TX, deliver the READY handshake and payload
    _gatt_db.TXChar.setEventHandler(BLESubscribed, ExoBLE::_on_tx_subscribed);


    exo_wdt_feed();   //BLE.begin() and the GATT registration above are behind us

    //CONNECTION INTERVAL EXPERIMENT, 2026-09-11. Was `setConnectionInterval(6, 6)`.
    //
    //Units are 1.25 ms, so (6, 6) pinned the link at 7.5 ms - the BLE MINIMUM - as both the min AND
    //the max, giving the central no range to negotiate within. That demands 133 connection events
    //per second, every second, from a Windows host that is also scheduling its own radio work.
    //
    //WHY CHANGE IT: by 2026-09-11 the failure was characterised as the BLE link dying every 20 s to
    //13 min, after which the controller stops completing packets (see the doc, section 3.9). Every
    //fix so far addressed what happens AFTER that; this is the first change aimed at why it happens.
    //A rigid 7.5 ms interval is the most aggressive thing this firmware asks of the link.
    //
    //(12, 24) = 15-30 ms. The RT stream is ~100 notifications/s, which needs ~1.5-3 packets per
    //connection event in this range - normal for a BLE central, but NOT something this setup has
    //ever exercised, because at 7.5 ms it only ever needed <1 packet per event.
    //
    //WATCH FOR: if the RT stream thins out (gaps in exo time in the CSV, plots updating slower than
    //100 Hz) then the host is NOT sending multiple packets per event, and the interval is throttling
    //throughput. In that case either revert to (6, 6), or try (6, 24) - which keeps 7.5 ms available
    //as the minimum but lets the host back off to 30 ms when it is busy, testing "rigid" rather than
    //"fast" as the problem.
    //
    //---- A/B/A CONTROL EXPERIMENT, 2026-09-12 (doc section 14.3) --------------------------------
    //Which arm is compiled is set by EXO_BLE_INTERVAL_PINNED in SystemReset.h, NOT here, because
    //the build tag in the banner is derived from that same symbol - so a log can never misreport
    //which arm produced it. Flip it there; this file follows.
    //
    //  arm A' (=1): (6, 6)   - put the failing configuration BACK. Expect failure inside ~4 min.
    //  arm B  (=0): (12, 24) - the candidate fix.
    //
    //Why bother: section 9's evidence is entirely between-groups and the two groups are separated
    //by several days AND a library patch AND the ping AND a host reboot. If the failure returns on
    //arm A' and then goes away again on arm B, the interval is the only thing that moved. If it
    //does NOT return, something else was doing the work and section 9 needs rewriting.
    if (EXO_BLE_INTERVAL_PINNED) {
        BLE.setConnectionInterval(6, 6);
    } else {
        BLE.setConnectionInterval(12, 24);
    }

    //No-op unless EXO_CRASH_TRAP_SELFTEST is 1 in SystemReset.h. Placed last so a self-test fault
    //happens after the reset-reason string is already parked in ErrorChar, and before advertising -
    //there is no point advertising on a boot we are about to deliberately end.
    exo_crash_trap_selftest();

    advertising_onoff(true);

    return true;
}

void ExoBLE::advertising_onoff(bool onoff)
{
    if (onoff)
    {
        //Start Advertising
        // logger::println("Start Advertising");
        BLE.advertise();

        //Turn the blue led off
        ComsLed *led = ComsLed::get_instance();
        uint8_t r, g, b;
        led->get_color(&r, &g, &b);
        led->set_color(r, g, 0);
    }
    else
    {
        //Stop Advertising
        // logger::println("Stop Advertising");
        BLE.stopAdvertise();

        //Turn the blue led on
        ComsLed *led = ComsLed::get_instance();
        uint8_t r, g, b;
        led->get_color(&r, &g, &b);
        led->set_color(r, g, 255);
    }
}

bool ExoBLE::handle_updates()
{
    #if EXOBLE_DEBUG
        logger::print("ExoBLE::handle_updates:Start");
        logger::print("\n");
    #endif

    static Time_Helper *t_helper = Time_Helper::get_instance();
    static float update_context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(update_context);

    if (del_t > BLE_times::_update_delay)
    {
        del_t = 0;
        #if EXOBLE_DEBUG
            static float poll_context = t_helper->generate_new_context();
            static float poll_time = 0;
            static float connected_context = t_helper->generate_new_context();
            static float connected_time = 0;
        #endif

        //Poll for updates and check connection status
        #if EXOBLE_DEBUG
            logger::print("Poll for updates and check connection status");
            logger::print("\n");
        #endif

        BLE.poll();
        int32_t current_status = BLE.connected();

        //Link-stall detector. Runs BEFORE the unchanged-status early return below, because an
        //unchanged status is exactly the failure: the stack still believes it is connected.
        if ((current_status > 0) && s_ping_seen &&
            ((uint32_t)(millis() - s_last_rx_ms) > EXO_BLE_STALL_MS))
        {
            //Records EXO_STALL_MAGIC and warm-resets, so the next boot's banner says BLESTALL_n.
            //Warm keeps GPREGRET, which is what makes this self-confirming rather than a guess.
            exo_ble_stall_reset(exo_ble_link_diag());
        }

        if (_connected == current_status)
        {
            #if EXOBLE_DEBUG
                logger::print("ExoBLE::handle_updates:queue size:");
                logger::print(ble_queue::size());
                logger::print("\n");
            #endif

            return ble_queue::size();
        }

        //The BLE connection status changed
        if (current_status < _connected)
        {
            //Disconnection
            #if EXOBLE_DEBUG
                logger::print("Disconnection");
                logger::print("\n");
            #endif
        }
        else if (current_status > _connected)
        {
            //Connection
            #if EXOBLE_DEBUG
                logger::print("Connection");
                logger::print("\n");
            #endif

            // Mark connected; wait for TX subscribe before sending handshake.
            //Restart the liveness clock here, or a long gap spent advertising would trip the
            //detector the instant somebody connects.
            s_last_rx_ms = millis();
            _connected = current_status;
            _tx_subscribed = false;
            _handshake_sent_this_connection = false;
            _handshake_payload_pending = true;

            //Append the connection parameters the CENTRAL chose (doc section 14.2). They only exist
            //once the LE Connection Complete event has landed, which is why this cannot be done at
            //boot with the rest of the banner.
            //
            //SAFE TO WRITE HERE, for two reasons worth stating because ErrorChar is BLENotify:
            //  1. Nobody is subscribed yet. The GUI subscribes to ErrorChar only AFTER its one-shot
            //     read (QtExoDeviceManager: read at :365, start_notify at :392), so writeValue()
            //     stores locally and sends nothing. No packet is added to the connect burst - the
            //     burst that already loses controller rows.
            //  2. It lands before that read. This runs the first main-loop pass after the link comes
            //     up; the GUI's read is several seconds later. If it ever did lose the race the only
            //     cost is a banner without the CP field - degraded, never wrong.
            {
                String banner = s_boot_banner + exo_ble_cp_string();
                char banner_char[banner.length() + 1];
                banner.toCharArray(banner_char, banner.length() + 1);
                _gatt_db.ErrorChar.writeValue(banner_char);
            }
        }

        advertising_onoff(current_status == 0);
        _connected = current_status;
    }

    if (_connected > 0 &&
        _tx_subscribed &&
        !_handshake_sent_this_connection &&
        _handshake_payload_pending &&
        rxBuffer_bulkStr[0] != '\0')
    {
        if (send_handshake_payload(_gatt_db.TXChar))
        {
            _handshake_sent_this_connection = true;
            _handshake_payload_pending = false;
        }
    }

    #if EXOBLE_DEBUG
        logger::print("ExoBLE::handle_updates:queue size:");
        logger::print(ble_queue::size());
        logger::print("\n");
    #endif

    return ble_queue::size();
}

void ExoBLE::send_message(BleMessage &msg)
{
    if (!this->_connected)
    {
        return; /* Don't bother sending anything if no one is listening */
    }

    #if EXOBLE_DEBUG
        BleMessage::print(msg);
    #endif

    static const int k_preamble_length = 3;
    int max_payload_length = ((k_preamble_length + msg.expecting) * (MAX_PARSER_CHARACTERS + 1));
    byte buffer[max_payload_length];

    int bytes_to_send = _ble_parser.package_raw_data(buffer, msg);

    //Time the write. If ArduinoBLE's `while (_pendingPkt >= _maxPkt) poll();` (HCI.cpp:636) is
    //engaged this is where it shows up - the call stops returning promptly. Cheap enough to leave on
    //the 100 Hz RT path: two micros() reads and a compare.
    const uint32_t t0 = micros();
    const bool write_ok = _gatt_db.TXChar.writeValue(buffer, bytes_to_send);
    const uint32_t dt = micros() - t0;

    if (dt > s_max_write_us) { s_max_write_us = dt; }
    if (!write_ok)           { s_write_failed = true; }
    s_sends_since_rx++;
}

void ExoBLE::send_error(int error_code, int joint_id)
{
    if (!this->_connected)
    {
        return; /* Don't bother sending anything if no one is listening */
    }

    #if EXOBLE_DEBUG
        logger::print("Exoble::send_error->Sending: ", LogLevel::Error);
        logger::print(joint_id, LogLevel::Error);
        logger::print(", ", LogLevel::Error);
        logger::print(error_code, LogLevel::Error);
        logger::print("\n");
    #endif

    String error_string = String(error_code) + ":" + String(joint_id);
    
    //Convert to char array
    char error_char[error_string.length() + 1];
    error_string.toCharArray(error_char, error_string.length() + 1);

    _gatt_db.ErrorChar.writeValue(error_char);
}

void ExoBLE::_on_tx_subscribed(BLEDevice /*central*/, BLECharacteristic characteristic)
{
    //A subscribed GUI is the first moment we KNOW the link is genuinely working, so this is where the
    //consecutive-crash count gets cleared. Clearing it any earlier (at boot, say) would defeat the
    //boot-loop guard in mbed_error_hook, which relies on the count surviving a crash-reboot cycle.
    exo_crash_mark_healthy();

    if (_instance != nullptr)
    {
        _instance->_handle_tx_subscribed(characteristic);
    }
}

void ExoBLE::_handle_tx_subscribed(BLECharacteristic characteristic)
{
    _tx_subscribed = true;
    if (_connected <= 0 || _handshake_sent_this_connection)
    {
        return;
    }

    const bool delivered = send_handshake_payload(characteristic);
    _handshake_payload_pending = !delivered;
    if (delivered)
    {
        _handshake_sent_this_connection = true;
    }
}

void ble_rx::on_rx_recieved(BLEDevice central, BLECharacteristic characteristic)
{
    static BleMessage *empty_msg = new BleMessage();
    static BleParser *parser = new BleParser();
    static BleMessage *msg = new BleMessage();

    //Must reset message to avoid duplicate data
    (*msg) = *empty_msg;

    char data[255] = {0};
    int len = characteristic.valueLength();
    if (len > (int)sizeof(data))
    {
        len = sizeof(data);
    }
    characteristic.readValue(data, len);

    //Stamp before parsing: any byte arriving at all proves the link is alive end to end, whether or
    //not it turns out to be a command we recognise.
    s_last_rx_ms = millis();
    if ((len > 0) && (data[0] == ble_names::ping))
    {
        s_ping_seen = true;
    }

    //A byte arrived, so the link is demonstrably alive: start the send-path measurement over.
    s_max_write_us   = 0;
    s_sends_since_rx = 0;
    s_write_failed   = false;

        #if EXOBLE_DEBUG
            logger::print("On Rx Recieved: ");
            for (int i=0; i<len;i++)
            {
                logger::print(data[i]);
                logger::print(", ");
            }
            logger::print("\n");
        #endif

    msg = parser->handle_raw_data(data, len);
    
    if (msg->is_complete)
    {
        #if EXOBLE_DEBUG
            logger::print("on_rx_recieved->Command: ");
            BleMessage::print(*msg);
        #endif

        ble_queue::push(msg);
    }

    #if EXOBLE_DEBUG
        logger::print("on_rx_recieved->End\n");
    #endif
}

#endif // defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
