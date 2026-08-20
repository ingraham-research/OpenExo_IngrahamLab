#ifndef PLOTTINGTITLES_H
#define PLOTTINGTITLES_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "StatusDefs.h"
#include "Config.h"

//Specific Libraries
#include "ParseIni.h"
#include "ParamsFromSD.h"
#include "ListCtrlParams.h"
#include "RealTimeI2C.h"    //rt_data::*_RT_LEN - the channel count must match the payload length


#include <cstddef> // Required for size_t
#include <cstdint> // Required for uint8_t

// Maximum buffer size estimate (must be large enough for "t," + every column + its commas +
// "\n,??" + null terminator). The longest set is bilateral_ankle: 13 labels, 214 chars of text
// plus 12 commas plus 6 of markers/terminator = ~233.
const size_t MAX_COMBINED_HEADER_LENGTH = 400;

// --- Definition of the Mapping Function (INLINE, DYNAMIC) ---

/**
 * @brief Returns the column header string based on the configuration ID and 
 * the 0-based column index (0 to 10).
 */
inline const char* getColumnHeader(uint8_t column_index, uint8_t* config_to_send) {
    //using namespace config_defs;

    // Outer switch: Selects the set of headers based on the Exo Type ID
    switch (config_to_send[config_defs::exo_name_idx]) {
        
        case (uint8_t)config_defs::exo_name::bilateral_ankle:
        {
            // Inner switch: Selects the specific column name for this mode (0-based)
            switch (column_index) {
                case 0:  return "Desired Torque (L)";
                case 1:  return "Measured Torque (L)";
                case 2:  return "Desired Torque (R)";
                case 3:  return "Measured Torque (R)";
                case 4:  return "Toe FSR (L)";
                case 5:  return "In Stance (L)";
                case 6:  return "Toe FSR (R)";
                case 7:  return "In Stance (R)";
                //Final commanded torque at the JOINT: post-feed-forward, post-PID, post-clamp, and
                //what actually went out on CAN (0 on cycles that transmitted a zero frame).
                //Columns 0/2 ("Desired Torque") are the PRE-PID setpoint - the gap between 0 and 8
                //is the PID's contribution. Plotted one per panel (ActiveTrialPage._PLOT_PAGES).
                case 8:  return "Commanded Torque (L)";
                case 9:  return "Commanded Torque (R)";
                case 10: return "Status";   //HIJACK: was an unused constant. Now the exo status (see uart_commands.h bilateral_ankle data[10]).
                case 11: return "Exoskeleton time (seconds)";
                case 12: return "Battery Level (Volts)";
                default: return "INVALID_COL";
            }
        }
		
        case (uint8_t)config_defs::exo_name::bilateral_hip:
        {
            // Inner switch: Selects the specific column name for this mode (0-based)
            switch (column_index) {
                case 0:  return "Measured Torque (R)";
                case 1:  return "Desired Torque (R)";
                case 2:  return "Measured Torque (L)";
                case 3:  return "Desired Torque (L)";
                case 4:  return "Gait/100 (R)";
                case 5:  return "Toe FSR (R)";
                case 6:  return "Gait/100 (L)";
                case 7:  return "Toe FSR (L)";
                case 8:  return "Heel FSR (R)";
                case 9:  return "Heel FSR (L)";
                case 10: return "Battery Level (Volts)";
                default: return "INVALID_COL";
            }
        }

        case (uint8_t)config_defs::exo_name::bilateral_arm:
        {
            switch (column_index) {
                case 0:  return "Measured Torque Arm1 (R)";
                case 1:  return "Desired Torque Arm1 (R)";
                case 2:  return "Measured Torque Arm1 (L)";
                case 3:  return "Desired Torque Arm1 (L)";
                case 4:  return "Measured Torque Arm2 (R)";
                case 5:  return "Desired Torque Arm2 (R)";
                case 6:  return "Measured Torque Arm2 (L)";
                case 7:  return "Desired Torque Arm2 (L)";
                case 8:  return "Gait/100 (R)";
                case 9:  return "Gait/100 (L)";
                case 10: return "Battery Level (Volts)";
                default: return "INVALID_COL";
            }
        }
        
        // DEFAULT CASE: Generic Data Headers
        default:
        {
            switch (column_index) {
                case 0:  return "Channel 0";
                case 1:  return "Channel 1";
                case 2:  return "Channel 2";
                case 3:  return "Channel 3";
                case 4:  return "Channel 4";
                case 5:  return "Channel 5";
                case 6:  return "Channel 6";
                case 7:  return "Channel 7";
                case 8:  return "Channel 8";
                case 9:  return "Exoskeleton time (seconds)";
                case 10: return "Battery Level (Volts)";
                default: return "INVALID_COL";
            }
        }
    }
}


/**
 * @brief How many column labels getColumnHeader() defines for this exo configuration.
 *
 * MUST equal the matching rt_data::*_RT_LEN in RealTimeI2C.h, because the GUI treats this list
 * as the definition of the channel layout: it picks CSV columns by name, finds the battery and
 * status readouts by name, and resolves the plot x-axis from "Exoskeleton time (seconds)".
 *
 * create_plotting_titles() used to hardcode 11 here. When bilateral_ankle grew to 13 channels
 * this was not updated, so the last two labels were never advertised. The visible damage was
 * subtle and easy to blame on the controller: the CSV silently lost its exo-clock column, and
 * ActiveTrialPage fell back to plotting against wall-clock BLE arrival time, which clumps
 * samples into bursts and makes a perfectly steady signal look like it is jittering.
 */
inline size_t getColumnCount(uint8_t* config_to_send) {
    switch (config_to_send[config_defs::exo_name_idx]) {
        case (uint8_t)config_defs::exo_name::bilateral_ankle:
            return (size_t)rt_data::BILATERAL_ANKLE_RT_LEN;   // 13
        case (uint8_t)config_defs::exo_name::bilateral_hip:
            return (size_t)rt_data::BILATERAL_HIP_RT_LEN;     // 11
        case (uint8_t)config_defs::exo_name::bilateral_arm:
            return (size_t)rt_data::BILATERAL_ARM_RT_LEN;     // 11
        default:
            return 11;                                        // generic "Channel N" set
    }
}

/**
 * @brief Function declaration to combine the column strings into a single delimited C-string.
 */
//bool create_plotting_titles(char* output_buffer, size_t buffer_size, uint8_t* config_to_send);
void create_plotting_titles(uint8_t* config_to_send);

#endif
#endif


