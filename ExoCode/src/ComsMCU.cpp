#include "ComsMCU.h"
#include "StatusLed.h"
#include "StatusDefs.h"
#include "Time_Helper.h"
#include "UARTHandler.h"
#include "uart_commands.h"
#include "UART_msg_t.h"
#include "Config.h"
#include "error_codes.h"
#include "Logger.h"
#include "ComsLed.h"
#include "SystemReset.h"

#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

#define COMSMCU_DEBUG 0

ComsMCU::ComsMCU(ExoData* data, uint8_t* config_to_send):_data{data}
{
    /* switch (config_to_send[config_defs::battery_idx])
    {
    case (uint8_t)config_defs::battery::smart:
        _battery = new SmartBattery();
        break;
    case (uint8_t)config_defs::battery::dumb:
        _battery = new RCBattery();
        break;
    default:
        //logger::println("ERROR: ComsMCU::ComsMCU->Unrecognized battery type!");
        _battery = new RCBattery();
        break;
    } */

    // _battery->init();
    _exo_ble = new ExoBLE();
    _exo_ble->setup();

    uint8_t rt_data_len = 0;
    switch (config_to_send[config_defs::exo_name_idx])
    {
        case (uint8_t)config_defs::exo_name::bilateral_ankle:
            rt_data_len = rt_data::BILATERAL_ANKLE_RT_LEN;
            break;
        case (uint8_t)config_defs::exo_name::bilateral_hip:
            rt_data_len = rt_data::BILATERAL_HIP_RT_LEN;
            break;
        case (uint8_t)config_defs::exo_name::bilateral_hip_ankle:
            rt_data_len = rt_data::BILATERAL_HIP_ANKLE_RT_LEN;
            break;
        case (uint8_t)config_defs::exo_name::bilateral_arm:
            rt_data_len = rt_data::BILATERAL_ARM_RT_LEN;
            break;
        default:
            rt_data_len = 8;
            break;
    }

    //Remember the configured payload length. This used to be computed and then thrown away, and
    //update_gui() used rt_data::capacity (16) instead - which overran BleMessage::data[10] by six
    //floats on every packet and put three channels of uninitialised garbage on the BLE stream.
    //It is only the fallback: poll() returns the length carried by the packet itself.
    if (rt_data_len > 0 && rt_data_len <= rt_data::capacity)
    {
        _rt_len = rt_data_len;
    }

    // logger::print("ComsMCU::ComsMCU->rt_data_len: "); logger::println(rt_data_len);
}

void ComsMCU::handle_ble()
{
    #if COMSMCU_DEBUG
        logger::println("ComsMCU::handle_ble->Start");
    #endif

    bool non_empty_ble_queue = _exo_ble->handle_updates();

    if (non_empty_ble_queue)
    {
        #if COMSMCU_DEBUG
            logger::println("ComsMCU::handle_ble->non_empty_ble_queue");
        #endif

        BleMessage msg = ble_queue::pop();
        _process_complete_gui_command(&msg);

        #if COMSMCU_DEBUG
            logger::println("ComsMCU::handle_ble->processed message");
        #endif
    }

    #if COMSMCU_DEBUG
        logger::println("ComsMCU::handle_ble->End");
    #endif
}

void ComsMCU::local_sample()
{
    #if COMSMCU_DEBUG
        logger::println("ComsMCU::local_sample->Start");
    #endif

    Time_Helper* t_helper = Time_Helper::get_instance();
    static const float context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(context);

    if (del_t > (BLE_times::_status_msg_delay/2)) 
    {
        /* static float filtered_value = _battery->get_parameter();
        float raw_battery_value = _battery->get_parameter();
        filtered_value = utils::ewma(raw_battery_value, filtered_value, k_battery_ewma_alpha);
        _data->battery_value = filtered_value; */
        del_t = 0;
    }

    ComsLed::get_instance()->life_pulse();
    _maybe_system_reset();

    #if COMSMCU_DEBUG
        logger::println("ComsMCU::local_sample->End");
    #endif
}

void ComsMCU::update_UART()
{
    #if COMSMCU_DEBUG
        logger::println("ComsMCU::update_UART->Start");
    #endif

    static Time_Helper* t_helper = Time_Helper::get_instance();
    static const float _context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(_context);
    
    if (del_t > UART_times::UPDATE_PERIOD)
    {
        UARTHandler* handler = UARTHandler::get_instance();
        UART_msg_t msg = handler->poll(UART_times::COMS_MCU_TIMEOUT);

        if (msg.command)
        {
            if (msg.command == UART_command_names::reset_ack)
            {
                _reset_ack_received = true;   // Teensy confirmed it got the reset (relayed to GUI)
            }
            else if (msg.command == UART_command_names::update_controller_param_ack)
            {
                _send_param_update_ack(msg);
            }
            else
            {
                UART_command_utils::handle_msg(handler, _data, msg);
            }
        }

        del_t = 0;
    }

    #if COMSMCU_DEBUG
        logger::println("ComsMCU::update_UART->End");
    #endif
}


void ComsMCU::update_gui() 
{
    #if COMSMCU_DEBUG
        logger::println("ComsMCU::update_gui->Start");
    #endif

    static Time_Helper* t_helper = Time_Helper::get_instance();
    static float my_mark = _data->mark;
    // WAS: `new float(rt_data::len)` -- a single-float allocation, not an array, which
    // real_time_i2c::poll() then filled with rt_data::len floats. See RealTimeI2C.h.
    static float rt_floats[rt_data::MAX_LEN];

    //Get real time data from ExoData and send to GUI.
    //poll() reports how many floats it actually wrote; keep that as the authoritative length so
    //everything downstream (the copy loop, `expecting`, the BLE frame) agrees with the packet.
    uint8_t rt_len_from_packet = 0;
    const bool new_rt_data = real_time_i2c::poll(rt_floats, &rt_len_from_packet);
    if (new_rt_data && rt_len_from_packet > 0)
    {
        _rt_len = rt_len_from_packet;
    }
    static float del_t_no_msg = millis();

    if (new_rt_data || rt_data::new_rt_msg)
    {
        del_t_no_msg = millis();

        #if COMSMCU_DEBUG
            logger::println("ComsMCU::update_gui->new_rt_data");
        #endif

        _life_pulse();
        rt_data::new_rt_msg = false;

        //Length is the PAYLOAD length, never rt_data::capacity, and it is clamped to what
        //BleMessage can physically hold. BleMessage::data is _max_size floats; writing past it
        //corrupts whatever follows the object (here, a stack frame) at ~100 Hz.
        uint8_t rt_len = _rt_len;
        if (rt_len > (uint8_t)BleMessage::k_max_data)
        {
            rt_len = (uint8_t)BleMessage::k_max_data;
        }
        if (rt_len > rt_data::capacity)
        {
            rt_len = rt_data::capacity;
        }

        BleMessage rt_data_msg = BleMessage();
        rt_data_msg.command = ble_names::send_real_time_data;
        rt_data_msg.expecting = rt_len;

        for (uint8_t i = 0; i < rt_len; i++)
        {
            #if REAL_TIME_I2C
                rt_data_msg.data[i] = rt_floats[i];
            #else
                rt_data_msg.data[i] = rt_data::float_values[i];
            #endif
        }

        if (my_mark < _data->mark)
        {
            my_mark = _data->mark;
            rt_data_msg.data[_mark_index] = my_mark;
        }

        _exo_ble->send_message(rt_data_msg);

        #if COMSMCU_DEBUG
            logger::println("ComsMCU::update_gui->sent message");
        #endif
    } 
    else 
    {
        //If we should be getting messages and we dont for 1 second, spin on error
        uint16_t exo_status = _data->get_status();
        const bool correct_status = (exo_status == status_defs::messages::trial_on) || 
            (exo_status == status_defs::messages::fsr_calibration) || 
            (exo_status == status_defs::messages::fsr_refinement) ||
            (exo_status == status_defs::messages::error);

        if (correct_status)
        {
            // if (millis() - del_t_no_msg > 3000)
            // {
            //     #if COMSMCU_DEBUG
            //          logger::println("ComsMCU::update_gui->No message for 3 second");
            //     #endif
            //     while (true)
            //     {
            //         logger::println("ComsMCU::update_gui->No message for 3 second");
            //         delay(10000);
            //     }
            // }
        }
    }

    //Periodically send status information
    static float status_context = t_helper->generate_new_context(); 
    static float del_t_status = 0;
    del_t_status += t_helper->tick(status_context);
    if (del_t_status > BLE_times::_status_msg_delay)
    {
        #if COMSMCU_DEBUG
            logger::println("ComsMCU::update_gui->Sending status");
        #endif

        //Send status data
        /* BleMessage batt_msg = BleMessage();
        batt_msg.command = ble_names::send_batt;
        batt_msg.expecting = ble_command_helpers::get_length_for_command(batt_msg.command);
        batt_msg.data[0] = _data->battery_value;
        _exo_ble->send_message(batt_msg); */

        del_t_status = 0;

        #if COMSMCU_DEBUG
            logger::println("ComsMCU::update_gui->sent message");
        #endif
    }

    #if COMSMCU_DEBUG
        logger::println("ComsMCU::update_gui->End");
    #endif
}

void ComsMCU::handle_errors()
{
    #if COMSMCU_DEBUG
        logger::println("ComsMCU::handle_errors->Start");
    #endif

    static ErrorCodes error_code = NO_ERROR;

    if (_data->error_code != static_cast<int>(error_code))
    {
        error_code = static_cast<ErrorCodes>(_data->error_code);
        _exo_ble->send_error(_data->error_code, _data->error_joint_id);
    }

    #if COMSMCU_DEBUG
        logger::println("ComsMCU::handle_errors->End");
    #endif
}

void ComsMCU::_process_complete_gui_command(BleMessage* msg) 
{
    #if COMSMCU_DEBUG
        logger::println("ComsMCU::_process_complete_gui_command->Start");
        BleMessage::print(*msg);
    #endif

    switch (msg->command)
    {
    case ble_names::start:
        ble_handlers::start(_data, msg);
        break;
    case ble_names::stop:
        ble_handlers::stop(_data, msg);
        break;
    case ble_names::cal_trq:
        ble_handlers::cal_trq(_data, msg);
        break;
    case ble_names::cal_fsr:
        ble_handlers::cal_fsr(_data, msg);
        break;
    case ble_names::assist:
        ble_handlers::assist(_data, msg);
        break;
    case ble_names::resist:
        ble_handlers::resist(_data, msg);
        break;
    case ble_names::motors_on:
        ble_handlers::motors_on(_data, msg);
        break;
    case ble_names::motors_off:
        ble_handlers::motors_off(_data, msg);
        break;
    case ble_names::mark:
        ble_handlers::mark(_data, msg);
        break;
    case ble_names::new_fsr:
        ble_handlers::new_fsr(_data, msg);
        break;
    case ble_names::new_trq:
        ble_handlers::new_trq(_data, msg);
        break;
    case ble_names::update_param:
    {
        param_update::Request request;
        param_update::RejectionReason reason = ble_handlers::update_param(_data, msg, &request);
        if (reason != param_update::RejectionReason::accepted)
        {
            _send_param_update_ack(request, false, reason);
        }
        break;
    }
    case ble_names::reset_system:
        _schedule_system_reset();
        break;
    default:
        logger::println("ComsMCU::_process_complete_gui_command->No case for command!", LogLevel::Error);
        break;
    }

    #if COMSMCU_DEBUG
        logger::println("ComsMCU::_process_complete_gui_command->End");
    #endif
}

void ComsMCU::_send_param_update_ack(
    const param_update::Request& request,
    bool accepted,
    param_update::RejectionReason reason)
{
    BleMessage ack_msg = BleMessage();
    ack_msg.command = ble_names::param_update_ack;
    ack_msg.expecting = ble_command_helpers::get_length_for_command(ack_msg.command);
    ack_msg.is_complete = true;
    ack_msg.data[0] = request.joint_id;
    ack_msg.data[1] = request.controller_id;
    ack_msg.data[2] = request.param_index;
    ack_msg.data[3] = accepted ? 1.0f : 0.0f;
    ack_msg.data[4] = (float)((uint8_t)reason);
    _exo_ble->send_message(ack_msg);
}

void ComsMCU::_send_param_update_ack(UART_msg_t msg)
{
    param_update::Request request;
    request.joint_id = msg.joint_id;
    uint8_t accepted_raw = 0;
    uint8_t reason_raw = (uint8_t)param_update::RejectionReason::malformed;

    bool parsed = (msg.len == (uint8_t)UART_command_enums::controller_param_ack::LENGTH) &&
        param_update::try_float_to_uint8(
            msg.data[(uint8_t)UART_command_enums::controller_param_ack::CONTROLLER_ID],
            &request.controller_id) &&
        param_update::try_float_to_uint8(
            msg.data[(uint8_t)UART_command_enums::controller_param_ack::PARAM_INDEX],
            &request.param_index) &&
        param_update::try_float_to_uint8(
            msg.data[(uint8_t)UART_command_enums::controller_param_ack::ACCEPTED],
            &accepted_raw) &&
        param_update::try_float_to_uint8(
            msg.data[(uint8_t)UART_command_enums::controller_param_ack::REASON],
            &reason_raw);

    if (!parsed)
    {
        logger::println("ComsMCU::_send_param_update_ack rejected malformed UART ack", LogLevel::Warn);
        reason_raw = (uint8_t)param_update::RejectionReason::malformed;
        accepted_raw = 0;
    }

    _send_param_update_ack(
        request,
        accepted_raw != 0,
        (param_update::RejectionReason)reason_raw);
}

void ComsMCU::_schedule_system_reset()
{
    _reset_state = ResetState::PENDING;
    _reset_step_ms = millis();
    _reset_ack_received = false;
}

void ComsMCU::_maybe_system_reset()
{
    switch (_reset_state)
    {
    case ResetState::IDLE:
        return;

    case ResetState::PENDING:
        _send_shutdown_progress(shutdown_progress::RECEIVED);
        _reset_state = ResetState::SENT;
        _reset_step_ms = millis();
        break;

    case ResetState::SENT:
    {
        UARTHandler* uart_handler = UARTHandler::get_instance();
        UART_msg_t tx_msg;
        tx_msg.command = UART_command_names::get_system_reset;
        tx_msg.joint_id = 0;
        tx_msg.len = 0;
        uart_handler->UART_msg(tx_msg);
        _send_shutdown_progress(shutdown_progress::SENT);
        _reset_ack_received = false;
        _reset_state = ResetState::WAIT_ACK;
        _reset_step_ms = millis();
        break;
    }

    case ResetState::WAIT_ACK:
        if (_reset_ack_received)
        {
            _send_shutdown_progress(shutdown_progress::ACKED);
            _reset_state = ResetState::SEND_REBOOT;
            _reset_step_ms = millis();
        }
        else if ((millis() - _reset_step_ms) >= _reset_ack_timeout_ms)
        {
            _send_shutdown_progress(shutdown_progress::TIMEOUT);
            _reset_state = ResetState::SEND_REBOOT;
            _reset_step_ms = millis();
        }
        break;

    case ResetState::SEND_REBOOT:
        // Send REBOOTING in a SEPARATE loop iteration from ACKED/TIMEOUT above. Two
        // writeValue() calls in one iteration queue two notifications before BLE.poll()
        // pumps them, and under load the first (ACKED/TIMEOUT) gets overwritten/dropped --
        // which was hiding that step. One notification per iteration lets each be sent.
        _send_shutdown_progress(shutdown_progress::REBOOTING);
        _reset_state = ResetState::REBOOTING;
        _reset_step_ms = millis();
        break;

    case ResetState::REBOOTING:
        // Give the final BLE notification time to go out before the radio dies.
        if ((millis() - _reset_step_ms) >= _reset_flush_ms)
        {
            exo_system_reset();
        }
        break;
    }
}

void ComsMCU::_send_shutdown_progress(uint8_t step)
{
    // Nano -> GUI: one end-trial shutdown step (see shutdown_progress:: in ble_commands.h).
    // Built like the real-time-data message in update_gui(); data[i] is scaled *100 by the
    // BLE parser, and the GUI divides back by 100.
    BleMessage msg = BleMessage();
    msg.command = ble_names::send_shutdown_progress;
    msg.expecting = 1;
    msg.data[0] = (float)step;
    _exo_ble->send_message(msg);
}

void ComsMCU::_life_pulse()
{
    #if COMSMCU_DEBUG
        logger::println("ComsMCU::_life_pulse->Start");
    #endif

    static int count = 0;
    count++;

    if (count > k_pulse_count)
    {
        count = 0;
        digitalWrite(25, !digitalRead(25));
    }

    #if COMSMCU_DEBUG
        logger::println("ComsMCU::_life_pulse->End");
    #endif
}
#endif
