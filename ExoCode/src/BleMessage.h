/**
 * @file BleMessage.h
 * @author Chance Cuddeback
 * @brief Defines the BleMessage class used to hold command-data pairs exchanged between the GUI.
 * @date 2022-08-22
 *
 */

#ifndef BLEMESSAGE_H
#define BLEMESSAGE_H

// Capacity of BleMessage::data, in floats.
//
// MUST be >= rt_data::MAX_LEN (RealTimeI2C.h). ComsMCU::update_gui() fills this array with a
// real-time packet and ExoBLE/BleParser then walk it up to `expecting`; when the real-time
// payload grew past 10 this array did not, so every packet wrote past the end of the object
// (and past `_size`, which sits immediately after it) at ~100 Hz. Raising it here is the fix;
// update_gui() also clamps to k_max_data so the two can never disagree silently again.
// Kept as a plain constant rather than pulling in RealTimeI2C.h, because this header is also
// compiled for the Teensy where that namespace means something slightly different.
static const int _max_size = 16;

class BleMessage
{
public:
    //Public alias so callers can bound-check before writing into data[].
    static constexpr int k_max_data = _max_size;

    /**
     * @brief Construct a new Ble Message object
     *
     */
    BleMessage();

    /**
     * @brief Set the message back to its defaults
     *
     */
    void clear();

    /**
     * @brief Sets its values equal to another BleMessage
     *
     * @param n Message to copy
     */
    void copy(BleMessage *n);

    //GUI command
    char command = 0;

    //Number of parameters to expect with the command
    int expecting = 0;

    //Variable to indicate the message has all of its data
    bool is_complete = false;

    //Array to hold the message parameters
    float data[_max_size] = {0};

    /**
     * @brief Print the message values to the serial monitor
     *
     * @param msg Message to print
     */
    static void print(BleMessage msg);

    /**
     * @brief Check if two messages are matching
     *
     * @param msg1 One of the messages to check
     * @param msg2 One of the messages to check
     * @return int One if the messages match, Zero if they dont
     */
    static int matching(BleMessage msg1, BleMessage msg2);

private:
    //Current index of the data array
    int _size = 0;
};

#endif
