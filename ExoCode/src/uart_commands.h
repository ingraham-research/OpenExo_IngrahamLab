#ifndef UART_COMMANDS_H
#define UART_COMMANDS_H

#include "Arduino.h"
#include "UARTHandler.h"
#include "UART_msg_t.h"

#include "ParseIni.h"
#include "ExoData.h"
#include "JointData.h"
#include "ParamsFromSD.h"
#include "ParamUpdateValidation.h"
#include "HeelFsrConfig.h"
#include "Logger.h"
#include "RealTimeI2C.h"
#include "SystemReset.h"
#include "SdLogger.h"     // for SdLogger::close_active() in the reset path (Teensy-only inside)

/**
 * @brief Type to associate a command with an ammount of data
 *
 */

namespace UART_command_names
{
    /* Update_x must be get_x + 1 */
    static const uint8_t empty_msg = 0x00;
    static const uint8_t get_controller_params = 0x01;
    static const uint8_t update_controller_params = 0x02;
    static const uint8_t get_status = 0x03;
    static const uint8_t update_status = 0x04;
    static const uint8_t get_config = 0x05;
    static const uint8_t update_config = 0x06;
    static const uint8_t get_cal_trq_sensor = 0x07;
    static const uint8_t update_cal_trq_sensor = 0x08;
    static const uint8_t get_cal_fsr = 0x09;
    static const uint8_t update_cal_fsr = 0x0A;
    static const uint8_t get_refine_fsr = 0x0B;
    static const uint8_t update_refine_fsr = 0x0C;
    static const uint8_t get_motor_enable_disable = 0x0D;
    static const uint8_t update_motor_enable_disable = 0x0E;
    static const uint8_t get_motor_zero = 0x0F;
    static const uint8_t update_motor_zero = 0x10;
    static const uint8_t get_real_time_data = 0x11;
    static const uint8_t update_real_time_data = 0x12;
    static const uint8_t get_controller_param = 0x13;
    static const uint8_t update_controller_param = 0x14;
    static const uint8_t get_error_code = 0x15;
    static const uint8_t update_error_code = 0x16;
    static const uint8_t get_FSR_thesholds = 0x17;
    static const uint8_t update_FSR_thesholds = 0x18;
    static const uint8_t get_system_reset = 0x19;
    static const uint8_t update_system_reset = 0x1A;
    static const uint8_t update_controller_param_ack = 0x1B;
    static const uint8_t reset_ack = 0x1C;   // Teensy -> Nano: "got the reset, closing logs"
};

/**
 * @brief Holds all of the enums for the UART commands. The enums are used to properly index the data
 *
 */
namespace UART_command_enums
{
    enum class controller_params : uint8_t
    {
        CONTROLLER_ID = 0,
        PARAM_LENGTH = 1,
        PARAM_START = 2,
        LENGTH
    };
    enum class status : uint8_t
    {
        STATUS = 0,
        LENGTH
    };
    enum class cal_trq_sensor : uint8_t
    {
        CAL_TRQ_SENSOR = 0,
        LENGTH
    };
    enum class cal_fsr : uint8_t
    {
        CAL_FSR = 0,
        LENGTH
    };
    enum class refine_fsr : uint8_t
    {
        REFINE_FSR = 0,
        LENGTH
    };
    enum class motor_enable_disable : uint8_t
    {
        ENABLE_DISABLE = 0,
        LENGTH
    };
    enum class motor_zero : uint8_t
    {
        ZERO = 0,
        LENGTH
    };
    enum class controller_param : uint8_t
    {
        CONTROLLER_ID = 0,
        PARAM_INDEX = 1,
        PARAM_VALUE = 2,
        LENGTH
    };
    enum class controller_param_ack : uint8_t
    {
        CONTROLLER_ID = 0,
        PARAM_INDEX = 1,
        ACCEPTED = 2,
        REASON = 3,
        LENGTH
    };
    enum class real_time_data : uint8_t
    {

    };
    enum class get_error_code : uint8_t
    {
        ERROR_CODE = 0,
        LENGTH
    };
    enum class FSR_thresholds : uint8_t
    {
        LEFT_THRESHOLD = 0,
        RIGHT_THRESHOLD = 1,
        LENGTH
    };
    enum class system_reset : uint8_t
    {
        LENGTH
    };
};

/**
 * @brief Holds the handlers for all of the commands. The handler function types should be the same. The 'get'
 * handlers will respond with the appropriate command, the 'update' handlers will unpack the msg and pack
 * exo_data
 *
 */
namespace UART_command_handlers
{
    inline static void get_controller_params(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
         //logger::println("UART_command_handlers::update_controller_params->Fetching params with msg: ");
         //UART_msg_t_utils::print_msg(msg);

        JointData *j_data = exo_data->get_joint_with(msg.joint_id);
        if (j_data == NULL)
        {
            //logger::println("UART_command_handlers::get_controller_params->No joint with id =  ");
            //logger::print(msg.joint_id);
            //logger::println(" found");
            return;
        }

        msg.command = UART_command_names::update_controller_params;

        uint8_t param_length = j_data->controller.get_parameter_length();
        msg.len = param_length + (uint8_t)UART_command_enums::controller_params::LENGTH;
        msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID] = j_data->controller.controller;
        msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_LENGTH] = param_length;
        for (int i = 0; i < param_length; i++)
        {
            msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + i] = j_data->controller.parameters[i];
        }

        handler->UART_msg(msg);
    }
    inline static void update_controller_params(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
         logger::println("UART_command_handlers::update_controller_params->Got new params with msg: ");
         UART_msg_t_utils::print_msg(msg);

        JointData *j_data = exo_data->get_joint_with(msg.joint_id);
        if (j_data == NULL)
        {
            logger::println("UART_command_handlers::update_controller_params->No joint with id =  " + String(msg.joint_id) + " found");
            return;
        }

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        j_data->controller.controller = (uint8_t)msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID];
        set_controller_params(msg.joint_id, (uint8_t)msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID], (uint8_t)msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START], exo_data);
        //Serial.println("Updating Controller Params: " + String(msg.joint_id) + ", " + String((uint8_t)msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START]) + ", " + String(j_data->controller.controller));
#endif
    }

    inline static void get_status(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {

        UART_msg_t tx_msg;
        tx_msg.command = UART_command_names::update_status;
        tx_msg.joint_id = 0;
        tx_msg.len = (uint8_t)UART_command_enums::status::LENGTH;
        tx_msg.data[(uint8_t)UART_command_enums::status::STATUS] = exo_data->get_status();

        handler->UART_msg(tx_msg);
        // logger::println("UART_command_handlers::get_status->sent updated status");
    }

    inline static void update_status(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        exo_data->set_status(msg.data[(uint8_t)UART_command_enums::status::STATUS]);
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        if (msg.data[(uint8_t)UART_command_enums::status::STATUS] == status_defs::messages::trial_on)
        {
            //Set default parameters for each used joint
            exo_data->set_default_parameters();
        }
#endif
    }

    inline static void get_config(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        UART_msg_t tx_msg;
        tx_msg.command = UART_command_names::update_config;
        tx_msg.joint_id = 0;
        tx_msg.len = ini_config::number_of_keys;
        tx_msg.data[config_defs::board_name_idx] = exo_data->config[config_defs::board_name_idx];
        tx_msg.data[config_defs::battery_idx] = exo_data->config[config_defs::battery_idx];
        tx_msg.data[config_defs::board_version_idx] = exo_data->config[config_defs::board_version_idx];
        tx_msg.data[config_defs::exo_name_idx] = exo_data->config[config_defs::exo_name_idx];
        tx_msg.data[config_defs::exo_side_idx] = exo_data->config[config_defs::exo_side_idx];
        tx_msg.data[config_defs::hip_idx] = exo_data->config[config_defs::hip_idx];
        tx_msg.data[config_defs::knee_idx] = exo_data->config[config_defs::knee_idx];
        tx_msg.data[config_defs::ankle_idx] = exo_data->config[config_defs::ankle_idx];
        tx_msg.data[config_defs::elbow_idx] = exo_data->config[config_defs::elbow_idx];
        tx_msg.data[config_defs::arm_1_idx] = exo_data->config[config_defs::arm_1_idx];
        tx_msg.data[config_defs::arm_2_idx] = exo_data->config[config_defs::arm_2_idx];
        tx_msg.data[config_defs::hip_gear_idx] = exo_data->config[config_defs::hip_gear_idx];
        tx_msg.data[config_defs::knee_gear_idx] = exo_data->config[config_defs::knee_gear_idx];
        tx_msg.data[config_defs::ankle_gear_idx] = exo_data->config[config_defs::ankle_gear_idx];
        tx_msg.data[config_defs::elbow_gear_idx] = exo_data->config[config_defs::elbow_gear_idx];
        tx_msg.data[config_defs::arm_1_gear_idx] = exo_data->config[config_defs::arm_1_gear_idx];
        tx_msg.data[config_defs::arm_2_gear_idx] = exo_data->config[config_defs::arm_2_gear_idx];
        tx_msg.data[config_defs::exo_hip_default_controller_idx] = exo_data->config[config_defs::exo_hip_default_controller_idx];
        tx_msg.data[config_defs::exo_knee_default_controller_idx] = exo_data->config[config_defs::exo_knee_default_controller_idx];
        tx_msg.data[config_defs::exo_ankle_default_controller_idx] = exo_data->config[config_defs::exo_ankle_default_controller_idx];
        tx_msg.data[config_defs::exo_elbow_default_controller_idx] = exo_data->config[config_defs::exo_elbow_default_controller_idx];
        tx_msg.data[config_defs::exo_arm_1_default_controller_idx] = exo_data->config[config_defs::exo_arm_1_default_controller_idx];
        tx_msg.data[config_defs::exo_arm_2_default_controller_idx] = exo_data->config[config_defs::exo_arm_2_default_controller_idx];
        tx_msg.data[config_defs::hip_use_torque_sensor_idx] = exo_data->config[config_defs::hip_use_torque_sensor_idx];
        tx_msg.data[config_defs::knee_use_torque_sensor_idx] = exo_data->config[config_defs::knee_use_torque_sensor_idx];
        tx_msg.data[config_defs::ankle_use_torque_sensor_idx] = exo_data->config[config_defs::ankle_use_torque_sensor_idx];
        tx_msg.data[config_defs::elbow_use_torque_sensor_idx] = exo_data->config[config_defs::elbow_use_torque_sensor_idx];
        tx_msg.data[config_defs::arm_1_use_torque_sensor_idx] = exo_data->config[config_defs::arm_1_use_torque_sensor_idx];
        tx_msg.data[config_defs::arm_2_use_torque_sensor_idx] = exo_data->config[config_defs::arm_2_use_torque_sensor_idx];
        tx_msg.data[config_defs::hip_flip_motor_dir_idx] = exo_data->config[config_defs::hip_flip_motor_dir_idx];
        tx_msg.data[config_defs::knee_flip_motor_dir_idx] = exo_data->config[config_defs::knee_flip_motor_dir_idx];
        tx_msg.data[config_defs::ankle_flip_motor_dir_idx] = exo_data->config[config_defs::ankle_flip_motor_dir_idx];
        tx_msg.data[config_defs::elbow_flip_motor_dir_idx] = exo_data->config[config_defs::elbow_flip_motor_dir_idx];
        tx_msg.data[config_defs::arm_1_flip_motor_dir_idx] = exo_data->config[config_defs::arm_1_flip_motor_dir_idx];
        tx_msg.data[config_defs::arm_2_flip_motor_dir_idx] = exo_data->config[config_defs::arm_2_flip_motor_dir_idx];
        tx_msg.data[config_defs::hip_flip_torque_dir_idx] = exo_data->config[config_defs::hip_flip_torque_dir_idx];
        tx_msg.data[config_defs::knee_flip_torque_dir_idx] = exo_data->config[config_defs::knee_flip_torque_dir_idx];
        tx_msg.data[config_defs::ankle_flip_torque_dir_idx] = exo_data->config[config_defs::ankle_flip_torque_dir_idx];
        tx_msg.data[config_defs::elbow_flip_torque_dir_idx] = exo_data->config[config_defs::elbow_flip_torque_dir_idx];
        tx_msg.data[config_defs::arm_1_flip_torque_dir_idx] = exo_data->config[config_defs::arm_1_flip_torque_dir_idx];
        tx_msg.data[config_defs::arm_2_flip_torque_dir_idx] = exo_data->config[config_defs::arm_2_flip_torque_dir_idx];
        tx_msg.data[config_defs::hip_flip_angle_dir_idx] = exo_data->config[config_defs::hip_flip_angle_dir_idx];
        tx_msg.data[config_defs::knee_flip_angle_dir_idx] = exo_data->config[config_defs::knee_flip_angle_dir_idx];
        tx_msg.data[config_defs::ankle_flip_angle_dir_idx] = exo_data->config[config_defs::ankle_flip_angle_dir_idx];
        tx_msg.data[config_defs::elbow_flip_angle_dir_idx] = exo_data->config[config_defs::elbow_flip_angle_dir_idx];
        tx_msg.data[config_defs::arm_1_flip_angle_dir_idx] = exo_data->config[config_defs::arm_1_flip_angle_dir_idx];
        tx_msg.data[config_defs::arm_2_flip_angle_dir_idx] = exo_data->config[config_defs::arm_2_flip_angle_dir_idx];

        handler->UART_msg(tx_msg);
        // logger::println("UART_command_handlers::get_config->sent updated config");
    }
    inline static void update_config(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        // logger::println("UART_command_handlers::update_config->got message: ");
        UART_msg_t_utils::print_msg(msg);
        exo_data->config[config_defs::board_name_idx] = msg.data[config_defs::board_name_idx];
        exo_data->config[config_defs::battery_idx] = msg.data[config_defs::battery_idx];
        exo_data->config[config_defs::board_version_idx] = msg.data[config_defs::board_version_idx];
        exo_data->config[config_defs::exo_name_idx] = msg.data[config_defs::exo_name_idx];
        exo_data->config[config_defs::exo_side_idx] = msg.data[config_defs::exo_side_idx];
        exo_data->config[config_defs::hip_idx] = msg.data[config_defs::hip_idx];
        exo_data->config[config_defs::knee_idx] = msg.data[config_defs::knee_idx];
        exo_data->config[config_defs::ankle_idx] = msg.data[config_defs::ankle_idx];
        exo_data->config[config_defs::elbow_idx] = msg.data[config_defs::elbow_idx];
        exo_data->config[config_defs::arm_1_idx] = msg.data[config_defs::arm_1_idx];
        exo_data->config[config_defs::arm_2_idx] = msg.data[config_defs::arm_2_idx];
        exo_data->config[config_defs::hip_gear_idx] = msg.data[config_defs::hip_gear_idx];
        exo_data->config[config_defs::knee_gear_idx] = msg.data[config_defs::knee_gear_idx];
        exo_data->config[config_defs::ankle_gear_idx] = msg.data[config_defs::ankle_gear_idx];
        exo_data->config[config_defs::elbow_gear_idx] = msg.data[config_defs::elbow_gear_idx];
        exo_data->config[config_defs::arm_1_gear_idx] = msg.data[config_defs::arm_1_gear_idx];
        exo_data->config[config_defs::arm_2_gear_idx] = msg.data[config_defs::arm_2_gear_idx];
        exo_data->config[config_defs::exo_hip_default_controller_idx] = msg.data[config_defs::exo_hip_default_controller_idx];
        exo_data->config[config_defs::exo_knee_default_controller_idx] = msg.data[config_defs::exo_knee_default_controller_idx];
        exo_data->config[config_defs::exo_ankle_default_controller_idx] = msg.data[config_defs::exo_ankle_default_controller_idx];
        exo_data->config[config_defs::exo_elbow_default_controller_idx] = msg.data[config_defs::exo_elbow_default_controller_idx];
        exo_data->config[config_defs::exo_arm_1_default_controller_idx] = msg.data[config_defs::exo_arm_1_default_controller_idx];
        exo_data->config[config_defs::exo_arm_2_default_controller_idx] = msg.data[config_defs::exo_arm_2_default_controller_idx];
        exo_data->config[config_defs::hip_use_torque_sensor_idx] = msg.data[config_defs::hip_use_torque_sensor_idx];
        exo_data->config[config_defs::knee_use_torque_sensor_idx] = msg.data[config_defs::knee_use_torque_sensor_idx];
        exo_data->config[config_defs::ankle_use_torque_sensor_idx] = msg.data[config_defs::ankle_use_torque_sensor_idx];
        exo_data->config[config_defs::elbow_use_torque_sensor_idx] = msg.data[config_defs::elbow_use_torque_sensor_idx];
        exo_data->config[config_defs::arm_1_use_torque_sensor_idx] = msg.data[config_defs::arm_1_use_torque_sensor_idx];
        exo_data->config[config_defs::arm_2_use_torque_sensor_idx] = msg.data[config_defs::arm_2_use_torque_sensor_idx];
        exo_data->config[config_defs::hip_flip_motor_dir_idx] = msg.data[config_defs::hip_flip_motor_dir_idx];
        exo_data->config[config_defs::knee_flip_motor_dir_idx] = msg.data[config_defs::knee_flip_motor_dir_idx];
        exo_data->config[config_defs::ankle_flip_motor_dir_idx] = msg.data[config_defs::ankle_flip_motor_dir_idx];
        exo_data->config[config_defs::elbow_flip_motor_dir_idx] = msg.data[config_defs::elbow_flip_motor_dir_idx];
        exo_data->config[config_defs::arm_1_flip_motor_dir_idx] = msg.data[config_defs::arm_1_flip_motor_dir_idx];
        exo_data->config[config_defs::arm_2_flip_motor_dir_idx] = msg.data[config_defs::arm_2_flip_motor_dir_idx];
        exo_data->config[config_defs::hip_flip_torque_dir_idx] = msg.data[config_defs::hip_flip_torque_dir_idx];
        exo_data->config[config_defs::knee_flip_torque_dir_idx] = msg.data[config_defs::knee_flip_torque_dir_idx];
        exo_data->config[config_defs::ankle_flip_torque_dir_idx] = msg.data[config_defs::ankle_flip_torque_dir_idx];
        exo_data->config[config_defs::elbow_flip_torque_dir_idx] = msg.data[config_defs::elbow_flip_torque_dir_idx];
        exo_data->config[config_defs::arm_1_flip_torque_dir_idx] = msg.data[config_defs::arm_1_flip_torque_dir_idx];
        exo_data->config[config_defs::arm_2_flip_torque_dir_idx] = msg.data[config_defs::arm_2_flip_torque_dir_idx];
        exo_data->config[config_defs::hip_flip_angle_dir_idx] = msg.data[config_defs::hip_flip_angle_dir_idx];
        exo_data->config[config_defs::knee_flip_angle_dir_idx] = msg.data[config_defs::knee_flip_angle_dir_idx];
        exo_data->config[config_defs::ankle_flip_angle_dir_idx] = msg.data[config_defs::ankle_flip_angle_dir_idx];
        exo_data->config[config_defs::elbow_flip_angle_dir_idx] = msg.data[config_defs::elbow_flip_angle_dir_idx];
        exo_data->config[config_defs::arm_1_flip_angle_dir_idx] = msg.data[config_defs::arm_1_flip_angle_dir_idx];
        exo_data->config[config_defs::arm_2_flip_angle_dir_idx] = msg.data[config_defs::arm_2_flip_angle_dir_idx];
    }

    inline static void get_cal_trq_sensor(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
    }
    inline static void update_cal_trq_sensor(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        // logger::println("UART_command_handlers::update_cal_trq_sensor->Got Cal trq sensor");
        exo_data->start_pretrial_cal();
    }

    inline static void get_cal_fsr(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
    }
    inline static void update_cal_fsr(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        // logger::println("UART_command_handlers::update_cal_fsr->Got msg");
        exo_data->right_side.reset_fsr_calibration = true;
        exo_data->right_side.do_calibration_toe_fsr = 1;
        exo_data->left_side.reset_fsr_calibration = true;
        exo_data->left_side.do_calibration_toe_fsr = 1;
        if (heel_fsr_present())
        {
            exo_data->right_side.do_calibration_heel_fsr = 1;
            exo_data->left_side.do_calibration_heel_fsr = 1;
        }
    }

    inline static void get_refine_fsr(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
    }
    inline static void update_refine_fsr(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        // logger::println("UART_command_handlers::update_refine_fsr->Got msg");
        exo_data->right_side.do_calibration_refinement_toe_fsr = 1;
        exo_data->left_side.do_calibration_refinement_toe_fsr = 1;
        if (heel_fsr_present())
        {
            exo_data->right_side.do_calibration_refinement_heel_fsr = 1;
            exo_data->left_side.do_calibration_refinement_heel_fsr = 1;
        }
    }

    inline static void get_motor_enable_disable(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
    }
    inline static void update_motor_enable_disable(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        // logger::println("UART_command_handlers::update_motor_enable_disable->Got msg");
        exo_data->for_each_joint([](JointData *j_data, float *args)
                                 {if (j_data->is_used) j_data->motor.enabled = (bool)args[0]; },
                                 msg.data);
        exo_data->user_paused = !(bool)msg.data[0];
    }

    inline static void get_motor_zero(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
    }
    inline static void update_motor_zero(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
    }

    inline static void get_real_time_data(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg, uint8_t *config)
    {
        UART_msg_t rx_msg;
        rx_msg.command = UART_command_names::update_real_time_data;
        rx_msg.joint_id = 0;
        rx_msg.len = (uint8_t)rt_data::BILATERAL_ANKLE_RT_LEN; 

        // logger::println("config[config_defs::exo_name_idx] :: "); //Uncomment if you want to check that system is receiving correct config info
        // logger::println(config[config_defs::exo_name_idx]);

        //Plotting Guide [Mapping data value (o,1,2,etc.) to the color and tab of the Python GUI; Rule of Thumb: Even = Blue, Odd = Orange). 
        //Tab One
        //0 = Top Blue Line
        //1 = Top Orange Line
        //2 = Bottom Blue Line
        //3 = Bottom Orange Line
        
        //Tab 2
        //4 = Top Blue Line
        //5 = Top Orange Line
        //6 = Bottom Blue Line
        //7 = Bottom Orange Line
        
        //8 = Not Plotted, Will Save
        //9 = Not Plotted, Will Save

        //Note: Ankle and Hip are Configured for Step Controller, Elbow for the ElbowMinMax Controller, Multi-joint for their primary control schemes

        switch (config[config_defs::exo_name_idx])
        {
        case (uint8_t)config_defs::exo_name::bilateral_ankle:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_ANKLE_RT_LEN;
			//CHANNEL LAYOUT (keep in sync with PlottingTitles.h getColumnHeader/bilateral_ankle
			//and ActiveTrialPage._PLOT_PAGES). Channels 0-7 are UNCHANGED from the original
			//layout, so every existing CSV stays directly comparable:
			//  0-3   Desired vs Measured torque, per leg
			//  4-7   Toe FSR + In Stance, per FOOT (deliberately paired: the stance step change
			//        overlays the FSR trace on one panel, which is how they are read)
			//  8-9   Commanded torque L/R  (NEW - plotted one per panel, see _PLOT_PAGES)
			//  10-12 Status, exo clock, battery (not plotted; each has its own GUI readout)
			rx_msg.data[0] = exo_data->left_side.ankle.controller.desired_torque;
			rx_msg.data[1] = exo_data->left_side.ankle.controller.filtered_torque_reading;
			rx_msg.data[2] = exo_data->right_side.ankle.controller.desired_torque;
			rx_msg.data[3] = exo_data->right_side.ankle.controller.filtered_torque_reading;
			rx_msg.data[4] = exo_data->left_side.toe_fsr;
			rx_msg.data[5] = exo_data->left_side.toe_stance;
			rx_msg.data[6] = exo_data->right_side.toe_fsr;
			rx_msg.data[7] = exo_data->right_side.toe_stance;
			// heel FSR is unused on this build ([Sensors] heelFsrPresent = 0)

			//Channels 8/9: the FINAL commanded torque at the JOINT, in Nm. This is what actually
			//went out on CAN: post-feed-forward, post-PID, post-gain-schedule, post-
			//MAX_JOINT_TORQUE_NM clamp, and ZERO on any cycle where a zero frame was sent
			//(motor.t_ff is assigned in send_data()'s transmit branches for exactly this reason).
			//Channels 0/2 ("Desired Torque") are the PRE-PID feed-forward setpoint and do NOT show
			//what drives the motor -- the gap between 0 and 8 is the PID's contribution.
			//motor.t_ff is motor-frame Nm; x gearing puts it at the joint, so 0/2, 1/3 and 8/9 are
			//all directly comparable.
			rx_msg.data[8] = exo_data->left_side.ankle.motor.t_ff * exo_data->left_side.ankle.motor.gearing;
			rx_msg.data[9] = exo_data->right_side.ankle.motor.t_ff * exo_data->right_side.ankle.motor.gearing;

			//HIJACK: this slot was an unused constant placeholder (was "= 8"). Repurposed to stream
			//the exo status so the GUI/CSV can see the trial/calibration state (status_defs::messages:
			//2=trial_on, 4=torque_calibration, 5=fsr_calibration, 6=fsr_refinement, 7=motor_start_up).
			//Watching it settle at 2 (trial_on) marks FSR refinement finished (set in
			//Side::check_calibration).
			rx_msg.data[10] = (float)exo_data->get_status();
			rx_msg.data[11] = (float)millis()/1000;
			rx_msg.data[12] = exo_data->get_batt_info(0); //Excluded from the CSV by name, not index
			break;
		}

        case (uint8_t)config_defs::exo_name::bilateral_hip:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_HIP_RT_LEN;
            rx_msg.data[0] = exo_data->right_side.hip.controller.filtered_torque_reading;
            rx_msg.data[1] = exo_data->right_side.hip.controller.desired_torque;
            rx_msg.data[2] = exo_data->left_side.hip.controller.filtered_torque_reading;
            rx_msg.data[3] = exo_data->left_side.hip.controller.desired_torque;
            rx_msg.data[4] = exo_data->right_side.percent_gait / 100;
            rx_msg.data[5] = exo_data->right_side.toe_fsr;
            rx_msg.data[6] = exo_data->left_side.percent_gait / 100;
            rx_msg.data[7] = exo_data->left_side.toe_fsr;
            rx_msg.data[8] = exo_data->right_side.heel_fsr;
            rx_msg.data[9] = exo_data->left_side.heel_fsr;
			rx_msg.data[10] = exo_data->get_batt_info(0); //Not saved in the CSV file
            break;
		}

        case (uint8_t)config_defs::exo_name::bilateral_elbow:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_ELBOW_RT_LEN;
            rx_msg.data[0] = exo_data->right_side.elbow.controller.filtered_torque_reading; 
            rx_msg.data[1] = exo_data->right_side.elbow.controller.desired_torque;
            rx_msg.data[2] = exo_data->left_side.elbow.controller.filtered_torque_reading;
            rx_msg.data[3] = exo_data->left_side.elbow.controller.desired_torque;
            rx_msg.data[4] = exo_data->right_side.elbow.controller.FlexSense;
            rx_msg.data[5] = exo_data->right_side.elbow.controller.ExtenseSense;
            rx_msg.data[6] = exo_data->left_side.elbow.controller.FlexSense;
            rx_msg.data[7] = exo_data->left_side.elbow.controller.ExtenseSense;
            rx_msg.data[8] = 8;
			rx_msg.data[9] = (float)millis()/1000;
			rx_msg.data[10] = exo_data->get_batt_info(0); //Not saved in the CSV file
			//Channels 11/12: the FINAL commanded torque at the JOINT, in Nm.
			//This is what actually went out on CAN: post-feed-forward, post-PID, post-gain-schedule,
			//post-MAX_JOINT_TORQUE_NM clamp, and zero on any cycle where a zero frame was sent
			//(motor.t_ff is now assigned in send_data()'s transmit branches for exactly this reason).
			//Channels 0/2 ("Desired Torque") are the PRE-PID feed-forward setpoint and do NOT show
			//what drives the motor - the difference between them is the PID's contribution.
			//motor.t_ff is motor-frame Nm; x gearing puts it at the joint, comparable to
			//"Desired Torque" and "Measured Torque".
			rx_msg.data[11] = exo_data->left_side.ankle.motor.t_ff * exo_data->left_side.ankle.motor.gearing;
			rx_msg.data[12] = exo_data->right_side.ankle.motor.t_ff * exo_data->right_side.ankle.motor.gearing;
			break;
		}

        case (uint8_t)config_defs::exo_name::bilateral_hip_ankle:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_HIP_ANKLE_RT_LEN;
            rx_msg.data[0] = exo_data->right_side.ankle.controller.filtered_torque_reading; 
            rx_msg.data[1] = exo_data->right_side.ankle.controller.desired_torque;
            rx_msg.data[2] = exo_data->left_side.ankle.controller.filtered_torque_reading;
            rx_msg.data[3] = exo_data->left_side.ankle.controller.desired_torque;
            rx_msg.data[4] = exo_data->right_side.hip.controller.desired_torque;
            rx_msg.data[5] = exo_data->right_side.percent_gait / 100;
            rx_msg.data[6] = exo_data->left_side.hip.controller.desired_torque;
            rx_msg.data[7] = exo_data->left_side.percent_gait / 100;
            rx_msg.data[8] = exo_data->right_side.toe_fsr;
            rx_msg.data[9] = exo_data->left_side.toe_fsr;
			rx_msg.data[10] = exo_data->get_batt_info(0); //Not saved in the CSV file
			break;
		}

        case (uint8_t)config_defs::exo_name::bilateral_hip_elbow:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_HIP_ELBOW_RT_LEN;
            rx_msg.data[0] = exo_data->right_side.elbow.controller.filtered_torque_reading;
            rx_msg.data[1] = exo_data->right_side.elbow.controller.desired_torque;
            rx_msg.data[2] = exo_data->left_side.elbow.controller.filtered_torque_reading;
            rx_msg.data[3] = exo_data->left_side.elbow.controller.desired_torque;
            rx_msg.data[4] = exo_data->right_side.hip.controller.desired_torque;
            rx_msg.data[5] = exo_data->right_side.percent_gait / 100;
            rx_msg.data[6] = exo_data->left_side.hip.controller.desired_torque;
            rx_msg.data[7] = exo_data->left_side.percent_gait / 100;
            rx_msg.data[8] = exo_data->right_side.elbow.controller.FlexSense;
            rx_msg.data[9] = exo_data->left_side.elbow.controller.FlexSense;
			rx_msg.data[10] = exo_data->get_batt_info(0); //Not saved in the CSV file
			break;
		}

        case (uint8_t)config_defs::exo_name::bilateral_ankle_elbow:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_ANKLE_ELBOW_RT_LEN;
            rx_msg.data[0] = exo_data->right_side.ankle.controller.filtered_torque_reading;
            rx_msg.data[1] = exo_data->right_side.ankle.controller.desired_torque;
            rx_msg.data[2] = exo_data->left_side.ankle.controller.filtered_torque_reading;
            rx_msg.data[3] = exo_data->left_side.ankle.controller.desired_torque;
            rx_msg.data[4] = exo_data->right_side.elbow.controller.filtered_torque_reading;
            rx_msg.data[5] = exo_data->right_side.elbow.controller.desired_torque;
            rx_msg.data[6] = exo_data->left_side.elbow.controller.filtered_torque_reading;
            rx_msg.data[7] = exo_data->left_side.elbow.controller.desired_torque;
            rx_msg.data[8] = exo_data->right_side.toe_fsr;
            rx_msg.data[9] = exo_data->left_side.toe_fsr;
			rx_msg.data[10] = exo_data->get_batt_info(0); //Not saved in the CSV file
			break;
		}

        case (uint8_t)config_defs::exo_name::bilateral_arm:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_ARM_RT_LEN;
            rx_msg.data[0] = exo_data->right_side.arm_1.controller.filtered_torque_reading;
            rx_msg.data[1] = exo_data->right_side.arm_1.controller.desired_torque;
            rx_msg.data[2] = exo_data->left_side.arm_1.controller.filtered_torque_reading;
            rx_msg.data[3] = exo_data->left_side.arm_1.controller.desired_torque;
            rx_msg.data[4] = exo_data->right_side.arm_2.controller.filtered_torque_reading;
            rx_msg.data[5] = exo_data->right_side.arm_2.controller.desired_torque;
            rx_msg.data[6] = exo_data->left_side.arm_2.controller.filtered_torque_reading;
            rx_msg.data[7] = exo_data->left_side.arm_2.controller.desired_torque;
            rx_msg.data[8] = exo_data->right_side.percent_gait / 100;
            rx_msg.data[9] = exo_data->left_side.percent_gait / 100;
			rx_msg.data[10] = exo_data->get_batt_info(0); //Not saved in the CSV file
			break;
		}

        default:
		{
            rx_msg.len = (uint8_t)rt_data::BILATERAL_ANKLE_RT_LEN;
			//CHANNEL LAYOUT (keep in sync with PlottingTitles.h getColumnHeader/bilateral_ankle
			//and ActiveTrialPage._PLOT_PAGES). Channels 0-7 are UNCHANGED from the original
			//layout, so every existing CSV stays directly comparable:
			//  0-3   Desired vs Measured torque, per leg
			//  4-7   Toe FSR + In Stance, per FOOT (deliberately paired: the stance step change
			//        overlays the FSR trace on one panel, which is how they are read)
			//  8-9   Commanded torque L/R  (NEW - plotted one per panel, see _PLOT_PAGES)
			//  10-12 Status, exo clock, battery (not plotted; each has its own GUI readout)
			rx_msg.data[0] = exo_data->left_side.ankle.controller.desired_torque;
			rx_msg.data[1] = exo_data->left_side.ankle.controller.filtered_torque_reading;
			rx_msg.data[2] = exo_data->right_side.ankle.controller.desired_torque;
			rx_msg.data[3] = exo_data->right_side.ankle.controller.filtered_torque_reading;
			rx_msg.data[4] = exo_data->left_side.toe_fsr;
			rx_msg.data[5] = exo_data->left_side.toe_stance;
			rx_msg.data[6] = exo_data->right_side.toe_fsr;
			rx_msg.data[7] = exo_data->right_side.toe_stance;
			// heel FSR is unused on this build ([Sensors] heelFsrPresent = 0)

			//Channels 8/9: the FINAL commanded torque at the JOINT, in Nm. This is what actually
			//went out on CAN: post-feed-forward, post-PID, post-gain-schedule, post-
			//MAX_JOINT_TORQUE_NM clamp, and ZERO on any cycle where a zero frame was sent
			//(motor.t_ff is assigned in send_data()'s transmit branches for exactly this reason).
			//Channels 0/2 ("Desired Torque") are the PRE-PID feed-forward setpoint and do NOT show
			//what drives the motor -- the gap between 0 and 8 is the PID's contribution.
			//motor.t_ff is motor-frame Nm; x gearing puts it at the joint, so 0/2, 1/3 and 8/9 are
			//all directly comparable.
			rx_msg.data[8] = exo_data->left_side.ankle.motor.t_ff * exo_data->left_side.ankle.motor.gearing;
			rx_msg.data[9] = exo_data->right_side.ankle.motor.t_ff * exo_data->right_side.ankle.motor.gearing;

			//HIJACK: this slot was an unused constant placeholder (was "= 8"). Repurposed to stream
			//the exo status so the GUI/CSV can see the trial/calibration state (status_defs::messages:
			//2=trial_on, 4=torque_calibration, 5=fsr_calibration, 6=fsr_refinement, 7=motor_start_up).
			//Watching it settle at 2 (trial_on) marks FSR refinement finished (set in
			//Side::check_calibration).
			rx_msg.data[10] = (float)exo_data->get_status();
			rx_msg.data[11] = (float)millis()/1000;
			rx_msg.data[12] = exo_data->get_batt_info(0); //Excluded from the CSV by name, not index
			break;
		}
        }

        #if REAL_TIME_I2C
        real_time_i2c::msg(rx_msg.data, rx_msg.len);
        #else
        handler->UART_msg(rx_msg);
        #endif

        //Serial.print("RX_Message: ");
        //UART_msg_t_utils::print_msg(rx_msg);
        //Serial.print("\n");

         //logger::println("UART_command_handlers::get_real_time_data->sent real time data");   //Uncomment if you want to test to see what data is being sent
         //UART_msg_t_utils::print_msg(rx_msg);
    }

    //Overload for no config
    inline static void get_real_time_data(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        uint8_t empty_config[ini_config::number_of_keys] = {0};
        get_real_time_data(handler, exo_data, msg, empty_config);
    }

    inline static void update_real_time_data(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        // logger::println("UART_command_handlers::update_real_time_data->got message: ");
        // UART_msg_t_utils::print_msg(msg);
        #if REAL_TIME_I2C
                return;
        #endif
        //Bound-check against the buffer CAPACITY instead of demanding an exact match. The old test
        //was `rt_data::len != msg.len`, where `len` was the capacity - so it rejected every
        //correctly sized message, exactly like the I2C guard did. Copy what arrived, clamped.
        //
        //NOTE: this UART fallback is dead while REAL_TIME_I2C is 1, and it has a second latent
        //problem: rt_data::float_values / new_rt_msg are `static` in a header, so every
        //translation unit gets its own copy. The writer here and the reader in ComsMCU.cpp are
        //different TUs, so this path cannot actually deliver data until those become extern.
        if (msg.len == 0 || msg.len > rt_data::capacity)
        {
            return;
        }
        for (int i = 0; i < msg.len; i++)
        {
            rt_data::float_values[i] = msg.data[i];
        }
        rt_data::new_rt_msg = true;
    }

    inline static void send_controller_param_ack(
        UARTHandler *handler,
        const param_update::Request& request,
        bool accepted,
        param_update::RejectionReason reason)
    {
        if (handler == NULL)
        {
            return;
        }

        UART_msg_t ack_msg;
        ack_msg.command = UART_command_names::update_controller_param_ack;
        ack_msg.joint_id = request.joint_id;
        ack_msg.len = (uint8_t)UART_command_enums::controller_param_ack::LENGTH;
        ack_msg.data[(uint8_t)UART_command_enums::controller_param_ack::CONTROLLER_ID] = request.controller_id;
        ack_msg.data[(uint8_t)UART_command_enums::controller_param_ack::PARAM_INDEX] = request.param_index;
        ack_msg.data[(uint8_t)UART_command_enums::controller_param_ack::ACCEPTED] = accepted ? 1.0f : 0.0f;
        ack_msg.data[(uint8_t)UART_command_enums::controller_param_ack::REASON] = (float)((uint8_t)reason);
        handler->UART_msg(ack_msg);
    }

    inline static void update_controller_param(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        param_update::Request request;
        request.joint_id = msg.joint_id;

        if (msg.len != (uint8_t)UART_command_enums::controller_param::LENGTH ||
            !param_update::try_float_to_uint8(
                msg.data[(uint8_t)UART_command_enums::controller_param::CONTROLLER_ID],
                &request.controller_id) ||
            !param_update::try_float_to_uint8(
                msg.data[(uint8_t)UART_command_enums::controller_param::PARAM_INDEX],
                &request.param_index))
        {
            logger::println("UART_command_handlers::update_controller_param rejected malformed message", LogLevel::Warn);
            send_controller_param_ack(handler, request, false, param_update::RejectionReason::malformed);
            return;
        }
        request.value = msg.data[(uint8_t)UART_command_enums::controller_param::PARAM_VALUE];

        JointData *j_data = NULL;
        param_update::RejectionReason reason = param_update::validate_request(exo_data, request, &j_data);
        if (reason != param_update::RejectionReason::accepted)
        {
            param_update::log_rejection("UART_command_handlers::update_controller_param", request, reason);
            send_controller_param_ack(handler, request, false, reason);
            return;
        }

        //Set the controller
        if (request.controller_id != j_data->controller.controller)
        {
            j_data->controller.controller = request.controller_id;
            exo_data->set_default_parameters((uint8_t)j_data->id);
        }

        //Set the parameter
        j_data->controller.parameters[request.param_index] = request.value;

        logger::print("UART_command_handlers::update_controller_param accepted: joint=");
        logger::print(request.joint_id);
        logger::print(", controller=");
        logger::print(request.controller_id);
        logger::print(", index=");
        logger::print(request.param_index);
        logger::print(", value=");
        logger::println(request.value);
        send_controller_param_ack(handler, request, true, param_update::RejectionReason::accepted);
		
		#ifdef SIMPLE_DEBUG
		Serial.print("\nTeensy just updated a control parameter:");
		Serial.print("\n***joint_id: ");
		Serial.print(msg.joint_id);
		Serial.print(", CONTROLLER_ID: ");
		Serial.print(j_data->controller.controller);
		Serial.print(", PARAM_INDEX: ");
		Serial.print(request.param_index);
		Serial.print(", PARAM_VALUE: ");
		Serial.print(j_data->controller.parameters[request.param_index]);
		#endif
		
        // Serial.println("Updating Controller Params: " + String(msg.joint_id) + ", "
        // + String((uint8_t)msg.data[(uint8_t)UART_command_enums::controller_param::CONTROLLER_ID]) + ", "
        // + String((uint8_t)msg.data[(uint8_t)UART_command_enums::controller_param::PARAM_INDEX]) + ", "
        // + String((uint8_t)msg.data[(uint8_t)UART_command_enums::controller_param::PARAM_VALUE]) + ", ");
    }

    inline static void update_controller_param_ack(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        (void)handler;
        (void)exo_data;
        (void)msg;
    }

    inline static void update_error_code(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        //logger::println("UART_command_handlers::update_error_code->got message: ");
        //UART_msg_t_utils::print_msg(msg);
        
        //Set the error code
        exo_data->error_code = msg.data[(uint8_t)UART_command_enums::get_error_code::ERROR_CODE];
        exo_data->error_joint_id = msg.joint_id;
    }

    inline static void get_error_code(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        logger::print("Sending error: ", LogLevel::Error);
        logger::print(msg.joint_id, LogLevel::Error);
        logger::print(", ", LogLevel::Error);
        logger::println(msg.data[0], LogLevel::Error);

        
        UART_msg_t tx_msg;
        tx_msg.command = UART_command_names::update_error_code;
        tx_msg.joint_id = msg.joint_id;
        tx_msg.len = 1;
        tx_msg.data[(uint8_t)UART_command_enums::get_error_code::ERROR_CODE] = msg.data[(uint8_t)UART_command_enums::get_error_code::ERROR_CODE];
        handler->UART_msg(tx_msg);
    }

    inline static void get_FSR_thesholds(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        UART_msg_t tx_msg;
        tx_msg.command = UART_command_names::update_FSR_thesholds;
        tx_msg.joint_id = 0;
        tx_msg.len = (uint8_t)UART_command_enums::FSR_thresholds::LENGTH;
        tx_msg.data[(uint8_t)UART_command_enums::FSR_thresholds::RIGHT_THRESHOLD] =
            (exo_data->right_side.toe_fsr_upper_threshold + exo_data->right_side.toe_fsr_lower_threshold) / 2;
        tx_msg.data[(uint8_t)UART_command_enums::FSR_thresholds::LEFT_THRESHOLD] =
            (exo_data->left_side.toe_fsr_upper_threshold + exo_data->left_side.toe_fsr_lower_threshold) / 2;
        handler->UART_msg(tx_msg);
        // logger::println("Sent FSR thresholds");
    }
    inline static void update_FSR_thesholds(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        logger::println("UART_command_handlers::update_FSR_thesholds->got message: ");
        logger::println(msg.data[(uint8_t)UART_command_enums::FSR_thresholds::RIGHT_THRESHOLD]);
        // UART_msg_t_utils::print_msg(msg);
        exo_data->right_side.toe_fsr_upper_threshold = msg.data[(uint8_t)UART_command_enums::FSR_thresholds::RIGHT_THRESHOLD] + fsr_config::SCHMITT_DELTA;
        exo_data->right_side.toe_fsr_lower_threshold = msg.data[(uint8_t)UART_command_enums::FSR_thresholds::RIGHT_THRESHOLD] - fsr_config::SCHMITT_DELTA;
        exo_data->left_side.toe_fsr_upper_threshold = msg.data[(uint8_t)UART_command_enums::FSR_thresholds::LEFT_THRESHOLD] + fsr_config::SCHMITT_DELTA;
        exo_data->left_side.toe_fsr_lower_threshold = msg.data[(uint8_t)UART_command_enums::FSR_thresholds::LEFT_THRESHOLD] - fsr_config::SCHMITT_DELTA;
    }

    // Request a system reset on the receiving MCU (used to reboot Teensy from Nano)
    inline static void get_system_reset(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        (void)msg;

        // Tell the Nano we received the reset FIRST, so the ack is on the wire before we die.
        // (The Nano relays this to the GUI as the ACKED shutdown step.)
        UART_msg_t ack;
        ack.command = UART_command_names::reset_ack;
        ack.joint_id = 0;
        ack.len = 0;
        handler->UART_msg(ack);

        // Put the system in a safe state, then DEFER the reboot by a few control cycles instead
        // of restarting the CPU here. Rebooting inline (the old behavior) short-circuited the
        // control loop: setting motor.enabled = 0 only flips a data flag, and the AK60v3's final
        // zero-torque CAN frame is emitted by run_side() -> _Motor::send_data() on the NEXT control
        // cycle. Restarting before that cycle ran left the (still-powered) AK60v3 holding its last
        // non-zero command through the reboot ("frozen" ankle). Arming reset_pending lets the
        // superloop (ExoCode.ino) run run_side() so the zero frame goes out, then it closes the SD
        // log and calls exo_system_reset(). The reset_ack above already went to the Nano, so the
        // ~few-ms slip is invisible to the Nano/GUI shutdown handshake.
        exo_data->for_each_joint([](JointData* j_data, float* args)
        {
            (void)args;
            j_data->motor.enabled = 0;
        #if END_TRIAL_CUTS_MOTOR_POWER
            // Also drop the physical motor-enable pin. `enabled = 0` only stops CAN frames; the
            // motor stays electrically live, so a bad frame can still be held. is_on is consumed
            // by _Motor::on_off() (called from run_joint every cycle), which drives the pin on the
            // is_on edge -- so the pin goes low on the NEXT control cycle, well inside the
            // reset_pending deferral and after send_data() has already emitted zero frames.
            // Nothing sets is_on back to true except the first-run init block in ExoCode.ino, so
            // this is only safe on a path that reboots. Do NOT copy it into motors_off ('w', the
            // Pause button) -- the motors would stay dead until a power cycle.
            j_data->motor.is_on = false;
        #endif
            return;
        });
        exo_data->set_status(status_defs::messages::trial_off);
        exo_data->reset_ticks = 0;
        exo_data->reset_pending = true;
    }

};

namespace UART_command_utils
{

    static UART_msg_t call_and_response(UARTHandler *handler, UART_msg_t msg, float timeout)
    {
        UART_msg_t rx_msg = {0, 0, 0, 0};
        uint8_t searching = 1;
        // logger::println("UART_command_utils::call_and_response->searching for message");
        float start_time = millis();
        while (searching)
        {
            handler->UART_msg(msg);
            // logger::println("UART_command_utils::call_and_response->sent msg");
            delay(500);
            rx_msg = handler->poll(200000);
            searching = (rx_msg.command != (msg.command + 1));

            if (millis() - start_time > timeout)
            {
                // logger::println("UART_command_utils::call_and_response->timed out");
                return rx_msg;
            }
        }
        // logger::println("UART_command_utils::call_and_response->found message:");
        UART_msg_t_utils::print_msg(rx_msg);
        return rx_msg;
    }

    static uint8_t get_config(UARTHandler *handler, uint8_t *config, float timeout)
    {
        UART_msg_t msg;
        float start_time = millis();
        while (1)
        {
            msg.command = UART_command_names::get_config;
            msg.len = 0;
            msg = call_and_response(handler, msg, timeout);

            if ((millis() - start_time) > timeout)
            {
                logger::println("UART_command_utils::get_config->timed out");
                return 1;
            }

            //The length of the message needs to be equal to the config length
            if (msg.len != ini_config::number_of_keys)
            {
                logger::println("UART_command_utils::get_config->msg.len != number_of_keys");
                //Keep trying to get config
                continue;
            }
            for (int i = 0; i < msg.len; i++)
            {
                //A valid config will not contain a zero
                if (!msg.data[i])
                {
                    logger::print("UART_command_utils::get_config->Config contained a zero at index ");
                    logger::println(i);

                    //Keep trying to get config
                    continue;
                }
            }
            logger::println("UART_command_utils::get_config->got good config");
            break;
        }

        //Pack config
        for (int i = 0; i < msg.len; i++)
        {
            config[i] = msg.data[i];
        }
        return 0;
    }

    static void wait_for_get_config(UARTHandler *handler, ExoData *data, float timeout)
    {
        UART_msg_t rx_msg;
        float start_time = millis();
        while (true)
        {
            logger::println("UART_command_utils::wait_for_config->Polling for config");
            rx_msg = handler->poll(100000);
            if (rx_msg.command == UART_command_names::get_config)
            {
                logger::println("UART_command_utils::wait_for_config->Got config request");
                UART_command_handlers::get_config(handler, data, rx_msg);
                break;
            }
            delayMicroseconds(500);

            if ((millis() - start_time) > timeout)
            {
                logger::println("UART_command_utils::wait_for_config->Timed out");
                return;
            }
        }
        logger::println("UART_command_utils::wait_for_config->Sent config");
    }

    static void handle_msg(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        if (msg.command == UART_command_names::empty_msg)
        {
            return;
        }

        //logger::println("UART_command_utils::handle_message->got message: ");
        //UART_msg_t_utils::print_msg(msg);

		// ---- Per-message Serial print removed (left here on purpose, see below) ----
		// Serial.print("\nmsg.command:");
		// Serial.print(msg.command);
		//
		// handle_msg() is called from the 500 Hz control loop on the Teensy and from the coms
		// loop on the Nano, once per inbound UART message. Normally that is a trickle, but any
		// condition that spams UART turns it into a torrent - the latched TorqueVarianceError,
		// for instance, reports every single control cycle, which is ~1000 messages/second and
		// therefore ~1000 blocking Serial writes/second right inside the control loop.
		// Kept commented because it is the fastest way to see what the two boards are saying to
		// each other, but it is not a practical debug channel any time soon: getting serial out
		// of this hardware means tethering a board that normally runs untethered on battery.
		// Prefer the BLE/RT stream or a rate-limited one-shot if this needs to be observed.
        switch (msg.command)
        {
        case UART_command_names::empty_msg:
            //logger::println("UART_command_utils::handle_message->Empty Message!");
            break;

        case UART_command_names::get_controller_params:
            UART_command_handlers::get_controller_params(handler, exo_data, msg);
            break;
        case UART_command_names::update_controller_params:
            UART_command_handlers::update_controller_params(handler, exo_data, msg);
            break;
        case UART_command_names::get_status:
            UART_command_handlers::get_status(handler, exo_data, msg);
            break;
        case UART_command_names::update_status:
            UART_command_handlers::update_status(handler, exo_data, msg);
            break;

        case UART_command_names::get_config:
            UART_command_handlers::get_config(handler, exo_data, msg);
            break;
        case UART_command_names::update_config:
            UART_command_handlers::update_config(handler, exo_data, msg);
            break;

        case UART_command_names::get_cal_trq_sensor:
            UART_command_handlers::get_cal_trq_sensor(handler, exo_data, msg);
            break;
        case UART_command_names::update_cal_trq_sensor:
            UART_command_handlers::update_cal_trq_sensor(handler, exo_data, msg);
            break;

        case UART_command_names::get_cal_fsr:
            UART_command_handlers::get_cal_fsr(handler, exo_data, msg);
            break;
        case UART_command_names::update_cal_fsr:
            UART_command_handlers::update_cal_fsr(handler, exo_data, msg);
            break;

        case UART_command_names::get_refine_fsr:
            UART_command_handlers::get_refine_fsr(handler, exo_data, msg);
            break;
        case UART_command_names::update_refine_fsr:
            UART_command_handlers::update_refine_fsr(handler, exo_data, msg);
            break;

        case UART_command_names::get_motor_enable_disable:
            UART_command_handlers::get_motor_enable_disable(handler, exo_data, msg);
            break;
        case UART_command_names::update_motor_enable_disable:
            UART_command_handlers::update_motor_enable_disable(handler, exo_data, msg);
            break;

        case UART_command_names::get_motor_zero:
            UART_command_handlers::get_motor_zero(handler, exo_data, msg);
            break;
        case UART_command_names::update_motor_zero:
            UART_command_handlers::update_motor_zero(handler, exo_data, msg);
            break;

        case UART_command_names::get_real_time_data:
            UART_command_handlers::get_real_time_data(handler, exo_data, msg);
            break;
        case UART_command_names::update_real_time_data:
            UART_command_handlers::update_real_time_data(handler, exo_data, msg);
            break;

        case UART_command_names::update_controller_param:
            UART_command_handlers::update_controller_param(handler, exo_data, msg);
            break;
        case UART_command_names::update_controller_param_ack:
            UART_command_handlers::update_controller_param_ack(handler, exo_data, msg);
            break;

        case UART_command_names::get_error_code:
            UART_command_handlers::get_error_code(handler, exo_data, msg);
            break;
        case UART_command_names::update_error_code:
            UART_command_handlers::update_error_code(handler, exo_data, msg);
            break;

        case UART_command_names::get_FSR_thesholds:
            UART_command_handlers::get_FSR_thesholds(handler, exo_data, msg);
            break;
        case UART_command_names::update_FSR_thesholds:
            UART_command_handlers::update_FSR_thesholds(handler, exo_data, msg);
            break;
        case UART_command_names::get_system_reset:
            UART_command_handlers::get_system_reset(handler, exo_data, msg);
            break;


        default:
            logger::println("UART_command_utils::handle_message->Unknown Message!", LogLevel::Error);
            UART_msg_t_utils::print_msg(msg);
            break;
        }
    }
};

#endif
