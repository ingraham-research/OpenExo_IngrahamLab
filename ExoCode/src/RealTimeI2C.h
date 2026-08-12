#ifndef REAL_TIME_I2C_H
#define REAL_TIME_I2C_H
#include "Arduino.h"

namespace rt_data
{
    // CAPACITY of any buffer that receives a real-time packet. This is NOT the payload length.
    // Must be >= the largest *_RT_LEN below, and the GUI already pads/truncates to 16
    // (Python_GUI/services/RtBridge.py), so 16 is the natural ceiling.
    static const uint8_t MAX_LEN = 16;

    static int BILATERAL_HIP_ANKLE_RT_LEN = 11;
    static int BILATERAL_ANKLE_RT_LEN = 13;   // 11 -> 13: added final commanded torque L/R (ch 11,12)
    static int BILATERAL_HIP_RT_LEN = 11;
    static int BILATERAL_ELBOW_RT_LEN = 11;
    static int BILATERAL_HIP_ELBOW_RT_LEN = 11;
    static int BILATERAL_ANKLE_ELBOW_RT_LEN = 11;
    static int BILATERAL_ARM_RT_LEN = 11;
    static const uint8_t len = MAX_LEN;

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
    void msg(float* data, int len);
    bool poll(float* pack_array);
    void init();
};

#endif
