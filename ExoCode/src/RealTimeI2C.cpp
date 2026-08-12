#include "RealTimeI2C.h"

#include "Config.h"
#include "Utilities.h"
#include "Logger.h"
#include <Wire.h>

//#define RT_I2C_DEBUG 1

#define FIXED_POINT_FACTOR 100

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    #define HOST 1
    #if BOARD_VERSION == AK_Board_V0_3
    #define MY_WIRE Wire1
    #else 
    #define MY_WIRE Wire
    #endif
#else if defined(ARDUINO_ARDUINO_NANO33BLE)
    #define HOST 0
    #define MY_WIRE Wire
#endif

#define RT_I2C_ADDR 0x02
#define RT_I2C_REG 0x02

static volatile bool new_bytes = false;

// Buffer CAPACITY in bytes: 2 preamble bytes + 2 bytes per float, sized from rt_data::capacity.
// This is the size of the buffer, NOT the length of any particular packet. The receive guard
// below must therefore accept a RANGE of lengths, not just this one value.
static const int byte_buffer_capacity = 2 + rt_data::capacity * (int)(sizeof(float)/sizeof(short int));

// Smallest sane packet: preamble + one float.
static const int byte_buffer_min = 2 + (int)(sizeof(float)/sizeof(short int));

// How many bytes the last accepted packet actually carried (set by the ISR, read by poll()).
static volatile uint8_t new_byte_count = 0;

// WAS: `new uint8_t(byte_buffer_len)` and `new float(rt_data::len)`.
// Both are single-object allocations, NOT arrays: `new uint8_t(24)` allocates ONE byte holding
// the value 24. The I2C receive ISR then wrote byte_buffer_len bytes into it and poll() memcpy'd
// byte_buffer_len bytes out of it, and float_values was filled with rt_data::len floats -- heap
// corruption on every real-time packet. Fixed arrays remove the heap from this path entirely.
static uint8_t byte_buffer[byte_buffer_capacity];
static float float_values[rt_data::capacity];

static uint8_t _packed_len(uint8_t len)
{
    uint8_t packed_len = 0;
    packed_len += (float)len * (sizeof(float)/sizeof(short int));
    packed_len += 2; //Preamble 
    return packed_len;
}

static void _pack(uint8_t msg_id, uint8_t len, float *data, uint8_t *data_to_pack)
{
    //Pack metadata.
    //data_to_pack[1] is the NUMBER OF FLOATS in this packet. It used to be written as `len + 2`
    //(a byte-ish count) while poll() consumed it as a float count, so poll() always read two
    //elements past the end of the payload. poll() now cross-checks this against the received
    //byte count and takes the smaller of the two, so it is correct either way, but the field
    //itself is now unambiguous: it is a float count.
    data_to_pack[0] = msg_id;
    data_to_pack[1] = len;

    //Convert float array to short int array
    uint8_t _num_bytes = sizeof(float)/sizeof(short int);
    uint8_t buf[_num_bytes];
    for (int i=0; i<len; i++)
    {
        utils::float_to_short_fixed_point_bytes(data[i], buf, FIXED_POINT_FACTOR);
        uint8_t _offset = (2) + _num_bytes*i;
        memcpy((data_to_pack + _offset), buf, _num_bytes);
    }
}

void real_time_i2c::msg(float* data, int len)
{
    //Clamp to what the buffers on both ends can actually carry. Silently overrunning either the
    //Wire transfer buffer (32 bytes) or the receiver's array is how this path failed before, so
    //bound it here once rather than trusting every caller.
    if (len <= 0)
    {
        return;
    }
    if (len > (int)rt_data::MAX_WIRE_PAYLOAD)
    {
        len = (int)rt_data::MAX_WIRE_PAYLOAD;
    }
    if (len > (int)rt_data::capacity)
    {
        len = (int)rt_data::capacity;
    }

    const uint8_t packed_len = _packed_len(len);
    uint8_t bytes[packed_len];
    _pack((uint8_t)RT_I2C_REG, (uint8_t)len, data, bytes);

    #if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        MY_WIRE.beginTransmission(RT_I2C_ADDR);
        MY_WIRE.send(bytes, packed_len);
        MY_WIRE.endTransmission();
    #endif
}

#if defined(ARDUINO_ARDUINO_NANO33BLE)
//Warning: This is an interrupt and needs to be kept as short as possible. (NO SERIAL PRINTS)
//Also, any variables that are stored outside of its scope must be marked volatile and need to have their concurrency managed
static void on_receive(int byte_len)
{
    // Accept any WELL-FORMED packet, not one exact size.
    //
    // This used to be `if (byte_len != byte_buffer_len) return;` where byte_buffer_len was derived
    // from the buffer CAPACITY (16 floats -> 34 bytes). The Teensy transmits the actual PAYLOAD
    // (13 floats -> 28 bytes), so the test never passed and every real-time packet was dropped:
    // blank GUI plots, an empty CSV, and no battery/status readout, with the exo still driving
    // torque. Bound the length instead, and let poll() work out the float count.
    if (byte_len < byte_buffer_min || byte_len > byte_buffer_capacity)
    {
        return;
    }
    if (((byte_len - 2) % (int)(sizeof(float)/sizeof(short int))) != 0)
    {
        return;   // payload is not a whole number of int16 fields
    }
    for (int i=0; i<byte_len; i++)
    {
        byte_buffer[i] = MY_WIRE.read();
    }
    new_byte_count = (uint8_t)byte_len;
    new_bytes = true;
}
#endif

void real_time_i2c::init()
{
    #if HOST
        MY_WIRE.begin();
    #else
        MY_WIRE.begin(RT_I2C_ADDR);
        MY_WIRE.onReceive(on_receive);
    #endif
}

bool real_time_i2c::poll(float* pack_array, uint8_t* out_len)
{
    #if RT_I2C_DEBUG
        logger::println("real_time_i2c::poll()->Start");
    #endif

    if (out_len != NULL)
    {
        *out_len = 0;
    }

    if (!new_bytes)
    {
        #if RT_I2C_DEBUG
            logger::println("real_time_i2c::poll()->End (no new bytes)");
        #endif
        return false;
    }

    noInterrupts();
    const uint8_t declared_len = byte_buffer[1];
    const uint8_t received_bytes = new_byte_count;
    uint8_t buff[byte_buffer_capacity];
    memcpy(buff, byte_buffer, byte_buffer_capacity);
    new_bytes = false;
    interrupts();

    // Derive the float count from the BYTES WE ACTUALLY RECEIVED, then take the smaller of that
    // and what the header claims, then clamp to the destination array. Deriving it from the byte
    // count is what makes this correct regardless of how the two boards' payload lengths drift,
    // and the clamp is the last line of defence for pack_array (rt_data::MAX_LEN floats).
    uint8_t len = (received_bytes >= 2)
                      ? (uint8_t)((received_bytes - 2) / (sizeof(float)/sizeof(short int)))
                      : 0;
    if (declared_len < len)
    {
        len = declared_len;
    }
    if (len > rt_data::capacity)
    {
        len = rt_data::capacity;
    }

    #if RT_I2C_DEBUG
        logger::print("real_time_i2c::poll()->Done copying bytes, len: ");
        logger::print(len);
        logger::println();
    #endif

    for (int i=0; i<(len); i++)
    {
        uint8_t data_offset = (2) + (i*2); //Preamble plus i * sizeof(float)/sizeof(short int)
        float tmp = 0;
        utils::short_fixed_point_bytes_to_float((uint8_t*)(buff+data_offset), &tmp, FIXED_POINT_FACTOR);
        pack_array[i] =  tmp;

        #if RT_I2C_DEBUG
            logger::print("real_time_i2c::poll()->i: ");
            logger::print(i);
            logger::print(" data_offset: ");
            logger::print(data_offset);
            logger::print(" buff[data_offset]: ");
            logger::print(buff[data_offset]);
            logger::print(" buff[data_offset+1]: ");
            logger::print(buff[data_offset+1]);
            logger::print(" tmp: ");
            logger::print(tmp);
            logger::print("\n");
        #endif
    }
    

    if (out_len != NULL)
    {
        *out_len = len;
    }

    #if RT_I2C_DEBUG
        logger::println("real_time_i2c::poll()->End");
    #endif
    return (len > 0);
}