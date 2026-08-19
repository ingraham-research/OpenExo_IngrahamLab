#ifndef REAL_TIME_I2C_H
#define REAL_TIME_I2C_H
#include "Arduino.h"

namespace rt_data
{
    // ============================ CAPACITY vs PAYLOAD LENGTH ============================
    // These two are NOT the same number and must never be confused again. They were, and it
    // silently killed the entire real-time stream:
    //
    //   CAPACITY  (MAX_LEN)          - how big every buffer is. Fixed, compile time.
    //   PAYLOAD   (*_RT_LEN)         - how many floats THIS exo config actually sends. Varies.
    //
    // The receive guard in RealTimeI2C.cpp used to be sized from a constant named `rt_data::len`
    // that was set to the CAPACITY (16 -> 34 bytes), while the Teensy transmitted the PAYLOAD
    // (13 -> 28 bytes). `if (byte_len != byte_buffer_len) return;` then dropped every single
    // packet, so the GUI got no plots, no CSV rows, no battery and no status - while the motors
    // ran normally. The ambiguous name `len` is gone; use `capacity` for buffers and carry the
    // real payload length WITH the packet (see _pack / poll).
    // ===================================================================================

    // CAPACITY of any buffer that receives a real-time packet. This is NOT the payload length.
    // Must be >= the largest *_RT_LEN below, and the GUI already pads/truncates to 16
    // (Python_GUI/services/RtBridge.py), so 16 is the natural ceiling.
    static const uint8_t MAX_LEN = 16;

    // PAYLOAD length per exo configuration. Keep each of these in sync with:
    //   - the channel list in uart_commands.h::get_real_time_data
    //   - the label list in PlottingTitles.h::getColumnHeader / getColumnCount
    // A mismatch between the first two breaks the stream; a mismatch with the third silently
    // strips channel names from the GUI (which drops CSV columns and the plot time axis).
    static int BILATERAL_HIP_ANKLE_RT_LEN = 11;
    static int BILATERAL_ANKLE_RT_LEN = 13;   // 11 -> 13: added final commanded torque L/R (ch 8,9)
    static int BILATERAL_HIP_RT_LEN = 11;
    static int BILATERAL_ELBOW_RT_LEN = 11;
    static int BILATERAL_HIP_ELBOW_RT_LEN = 11;
    static int BILATERAL_ANKLE_ELBOW_RT_LEN = 11;
    static int BILATERAL_ARM_RT_LEN = 11;

    // Buffer capacity, in floats. Named `capacity` on purpose: it is NOT a message length.
    static const uint8_t capacity = MAX_LEN;

    // HARD CEILING ON PAYLOAD LENGTH: an I2C packet is 2 preamble bytes + 2 bytes per float, and
    // both Wire implementations here use a 32-byte transfer buffer, so anything longer than
    // 15 floats is truncated on the wire. real_time_i2c::msg() clamps to this. If you ever need
    // more than 15 channels you must raise the Wire buffers on BOTH boards first.
    static const uint8_t MAX_WIRE_PAYLOAD = 15;

    // WAS: `static float* float_values = new float(len);`
    // That is `new float(11)` -- it allocates ONE float initialised to 11.0f, NOT an array of 11.
    // Every writer then ran off the end of a 4-byte allocation: real_time_i2c::poll() fills `len`
    // floats, and uart_commands.h:601 writes float_values[i] for i < len. That is a 40+ byte heap
    // overflow on every real-time packet, on the Nano, at ~100 Hz. A fixed array removes the heap
    // entirely. The same mistake existed in RealTimeI2C.cpp and ComsMCU.cpp and is fixed there too.
    static float float_values[MAX_LEN];

    static bool new_rt_msg = false;
};

namespace real_time_i2c
{
    /**
     * @brief Teensy -> coms MCU. Packs `len` floats as int16 fixed point and transmits.
     *        `len` is the PAYLOAD length (rt_data::*_RT_LEN), clamped to rt_data::MAX_WIRE_PAYLOAD.
     */
    void msg(float* data, int len);

    /**
     * @brief Coms MCU. Unpacks the newest packet into pack_array (must hold rt_data::MAX_LEN floats).
     *
     * @param pack_array destination, at least rt_data::MAX_LEN floats
     * @param out_len    receives how many floats were actually written. NEVER assume this is
     *                   rt_data::capacity - that assumption is what broke the stream before.
     * @return true if a new packet was unpacked
     */
    bool poll(float* pack_array, uint8_t* out_len);
    void init();
};

#endif
