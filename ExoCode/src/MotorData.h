/**
 * @file MotorData.h
 *
 * @brief Declares a class used to store data for motors to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/

#ifndef MotorData_h
#define MotorData_h

#include "Arduino.h"

#include "ParseIni.h"
#include "Board.h"

#include <stdint.h>

//Forward declaration
class ExoData;

class MotorData 
{
	public:
        MotorData(config_defs::joint_id id, uint8_t* config_to_send);
        
        /**
         * @brief reconfigures the the motor data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        config_defs::joint_id id;   /**< Motor id, should be the same as the joint id. */ 
        uint8_t motor_type;         /**< Type of motor being used. */
        float last_command;         /**< Last command sent to the motor. */
        float p;                    /**< Read position. */ 
        float v;                    /**< Read velocity. */
        float i;                    /**< Read current. */
        float kt;                   /**< Motor torque constant. */
        float p_des = 0;            /**< Desired position, not currently used but available for position control */
        float v_des = 0;            /**< Desired velocity, not currently used but available for velocity control */
        float kp = 0;               /**< Proportional gain */
        float kd = 0;               /**< Derivative gain */
        /**
         * @brief FULL torque command at the motor shaft [Nm], post-PID. NOT a feed-forward term.
         *
         * The name is a protocol name, not a control name: this value goes into the MIT frame's
         * `t_ff` field, which is feed-forward with respect to the MOTOR's own kp/kd impedance loop
         * (kp = kd = 0 here, so it is the entire command). It has nothing to do with the exo
         * controller's feed-forward term -- that is ControllerData::ff_setpoint / desired_torque.
         *
         * Provenance: Controller::calc_motor_cmd() returns `cmd = torque_cmd + _pid(...)`
         * -> JointData::controller.setpoint (Joint.cpp:652) -> / gearing -> Motor::send_data(),
         * which clamps it to MAX_JOINT_TORQUE_NM and only then assigns it here, inside the
         * transmit branches. So t_ff is post-PID, post-gain-schedule, post-clamp, and is exactly
         * what went on the wire -- 0 on cycles that sent a zero frame.
         * x gearing puts it at the joint (streamed that way on RT channels 8/9).
         */
        float t_ff = 0;
        
        bool do_zero;               /**< Flag to zero the position of the motor */
        bool enabled;               /**< Motor enable state*/
        bool is_on;                 /**< Motor power state */
        bool is_left;               /**< Motor side information 1 if on the left, 0 otherwise */
        bool flip_direction;        /**< Should the motor direction be flipped, if true torque commands and position/velocity information will be inverted */
        float gearing;              /**< Motor gearing used to convert motor position, velocity, and torque between the motor and joint frames. */

        //Timeout state
        int timeout_count;          /**< Number of timeouts in a row */
        int timeout_count_max = 40; /**< Number of timeouts in a row before the motor is disabled */
		
		//Real-time Maxon Motor Reset Feedback
		int maxon_plotting_scalar = 1;

};


#endif