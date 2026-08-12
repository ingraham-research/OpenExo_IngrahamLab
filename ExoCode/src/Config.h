/**
 * @file Config.h
 * @author Chancelor Cuddeback
 * @brief Configuration variables for the codebase.
 * @date 2023-07-18
 * 
 */

#ifndef Config_h
#define Config_h 

#include "Arduino.h"
#include "LogLevels.h"
	#define SIMPLE_DEBUG //Uncomment to enter SIMPLE_DEBUG mode. The exoskeleton’s operating status will be printed to the Serial Monitor.
    #define FIRMWARE_VERSION 0_1_0

    #define AK_Board_V0_1 1
    #define AK_Board_V0_3 2
    #define AK_Board_V0_4 3
    #define AK_Board_V0_5_1 4
	#define OpenExo_Board_V0_6_Maxon 5

    //Update the define below to match the board version being used
    #define BOARD_VERSION AK_Board_V0_5_1 //Update this define to match the board version being used

	#define BATTERY_SENSOR 3 //Set it to 0 to disable, 219 to use INA219, 260 to use INA260, 3 to use the OpenExo Board 0.5.1 Mark 3's onboard voltage divider
	#define CRITICAL_BATT_VAL 21 //In volts. Battery voltage below this will trigger the low battery warning in the GUI.
	#define RESISTOR_1 21250 //Set it to the measured resistance of R1 on the OpenExo Board 0.5.1 Mark 3, and update the volt_sense pin mapping in Board.h
	#define RESISTOR_2 3275 //Set it to the measured resistance of R2 on the OpenExo Board 0.5.1 Mark 3, and update the volt_sense pin mapping in Board.h
    #define REAL_TIME_I2C 1
    #define LOOP_FREQ_HZ 500
    #define LOOP_TIME_TOLERANCE 0.1 
    
    #define USE_SPEED_CHECK 0
	#define USE_ANGLE_SENSORS 1
	//Heel FSR presence is a RUNTIME setting: [Sensors] heelFsrPresent in config.ini (see HeelFsrConfig.h / heel_fsr_present()).

	//Absolute ceiling on commanded torque AT THE JOINT OUTPUT, in Nm. Enforced in
	//_CANMotor::send_data() as the final gate before the CAN frame is built, so it applies AFTER
	//feed-forward, PID, gain scheduling, and every controller. Non-finite commands are forced to 0
	//there too (constrain() is a macro and passes NaN straight through to a full-scale command).
	//This is a FAULT LIMIT, not a tuning knob: normal ankle assist peaks around 12-20 Nm, so 25
	//leaves headroom while making a runaway physically impossible. Raise only with a real reason.
	#define MAX_JOINT_TORQUE_NM 25.0f

	// ===================== ERROR MANAGER / ERROR REPORTER: TEMPORARILY DISABLED =====================
	// 0 = the whole error-detection-and-reporting framework is compiled out (current, deliberate).
	// 1 = re-enable it.
	//
	// !! DO NOT SET THIS BACK TO 1 UNTIL SOMETHING ACTUALLY CONSUMES THE ERRORS. !!
	//
	// Why it is off:
	//   * It protects nothing. Every handler in error_types.h has its `motor.enabled = false` line
	//     COMMENTED OUT, so a detected error takes no action whatsoever.
	//   * It cannot detect anything either. Five of the eight checks are hardcoded `return false`.
	//     MotorTimeoutError needs motor.timeout_count >= 40, but the only code that would increment
	//     it (_CANMotor::_handle_read_failure) is itself commented out. And TorqueVarianceError is
	//     MATHEMATICALLY INCAPABLE of firing: it tests whether the newest torque sample sits more
	//     than torque_std_dev_multiple (10) sample-standard-deviations from the mean of a window
	//     that CONTAINS that same sample. For a value inside its own sample of n, using the sample
	//     sd (utils::online_std_dev divides M2 by n-1), the maximum possible studentized deviate is
	//     (n-1)/sqrt(n). With torque_data_window_max_size = 100 that ceiling is 99/10 = 9.9, which
	//     is below the 10 threshold for ANY input whatsoever - verified numerically, including the
	//     pathological 99-equal-values-plus-one-huge-outlier case, which tops out at exactly 9.9.
	//     That leaves TorqueOutOfBoundsError (|torque| > 60 Nm) as the only reachable check, and
	//     only on a railed or faulted torque sensor, never in normal walking.
	//   * Nothing receives the output. The chain ends at ComsMCU::handle_errors -> one BLE
	//     notification on the Error characteristic -> QtExoDeviceManager::_on_error ->
	//     deviceErrorReceived, which MainWindow only connects INSIDE the `if remote.is_bound()`
	//     block. With no UDP subscriber attached the signal is dropped on the floor. It is not in
	//     the UI, not in the CSV, not even in the GUI log file.
	//   * So every control cycle it pays real time to reach a conclusion it can never reach. Per
	//     joint per cycle: two full copies of a 100-element std::queue<float> (utils::online_std_dev
	//     takes its argument BY VALUE and then copies it again internally), a 100-iteration Welford
	//     pass with a float divide per iteration, 8 std::map lookups and 8 virtual calls. Across two
	//     ankles that is ~4000 heap-allocating deque copies per second inside a hard real-time loop.
	//     Estimated on the order of 1% of the 2000 us cycle - NOT MEASURED, and small, but it is
	//     pure waste plus heap churn/fragmentation risk on a controller that runs for hours.
	//
	// The trap this switch is really guarding against: if someone later "fixes" the detector by
	// lowering torque_std_dev_multiple to something reachable, WITHOUT also fixing the rest, the
	// result is immediate and bad. torque_failure_count is incremented and NEVER reset, so the
	// error latches on permanently; Joint.cpp then reports it once per joint per cycle; and each
	// report ends in UARTHandler::UART_msg -> MY_SERIAL.flush(), which SPINS until the UART shift
	// register drains (~234 us for a 6-byte SLIP frame at 256000 baud 8N1). Two ankles at 500 Hz is
	// ~469 us of blocking per 2000 us cycle (~23%), and ~1000 messages/second into a coms MCU whose
	// ComsMCU::update_UART consumes at most ONE message per UART_times::UPDATE_PERIOD (1000 us).
	// That failure mode is latent today only because the threshold happens to be unreachable.
	//
	// What this does NOT disable: the real torque ceiling is MAX_JOINT_TORQUE_NM, enforced in
	// _CANMotor::send_data() as the final gate before the CAN frame, plus its non-finite rejection.
	// Those are untouched and remain the actual protection.
	//
	// Before re-enabling: give the handlers real actions, reset torque_failure_count, stop copying
	// the queue in online_std_dev, make the reporting non-blocking (or rate-limit it), and wire
	// deviceErrorReceived to something a human sees. A genuine torque cutout would be better
	// written as a direct check with a direct action than routed through this framework.
	#define ERROR_MANAGER_ENABLED 0

	//End Trial also drops the physical motor-enable pin (logic_micro_pins::enable_*_pin), not just
	//the `enabled` software flag. Belt-and-braces against any held CAN command surviving the reboot.
	//ON (1), BUT NOT YET BENCH-CHECKED -- verify before anyone wears the exo. It is not known
	//whether that pin cuts driver power (joint free-spins, safe) or asserts a driver disable that
	//SHORTS THE PHASES (velocity-dependent brake on both ankles at once, a fall hazard if End Trial
	//is pressed mid-stride). To check: power the exo, back-drive an ankle by hand, drop the pin,
	//and feel whether it goes free or stiff. Free -> keep this at 1. Stiff -> set it to 0.
	//See uart_commands.h::get_system_reset.
	//Note this path is untested code: the only other writer of is_on is the estop branch in
	//Motor.cpp::on_off(), and estop is hardcoded off in Exo.cpp, so it has likely never run.
	#define END_TRIAL_CUTS_MOTOR_POWER 1

    //MACRO magic to convert a define to a string
    #define VAL(str) #str
    #define TOSTRING(str) VAL(str)

    namespace logging
    {
        const LogLevel level = LogLevel::Release; //Release or Debug (Note: Enter Debug to have Logger print to serial monitor)
        const int baud_rate = 115200;
    }
    
    namespace sync_time
    {
        const unsigned int NUM_START_STOP_BLINKS = 1;                                   //The number of times to have the LED on during the start stop sequence
        const unsigned int SYNC_HALF_PERIOD_US = 125000;                                //Half blink period in micro seconds
        const unsigned int SYNC_START_STOP_HALF_PERIOD_US = 4 * SYNC_HALF_PERIOD_US;    //Half blink period for the begining and end of the sequence. This is usually longer so it is easy to identify.
    }

    namespace fsr_config
    {
        const float FSR_UPPER_THRESHOLD = 0.25;
        const float FSR_LOWER_THRESHOLD = 0.15;
        const float SCHMITT_DELTA = (FSR_UPPER_THRESHOLD - FSR_LOWER_THRESHOLD)/2;
    }
	
	namespace angle_sensor
	{
		const float ANGLE_UPPER_THRESHOLD = 0.9;
		const float ANGLE_LOWER_THRESHOLD = 0.1;
		const float ROM_LEFT = 103.2f;
		const float ROM_RIGHT = 91.4f; //In degrees
	}

    namespace analog
    {
        const float RESOLUTION = 12;    //The resolution of the analog to digital converter
        const float COUNTS = 4096;      //The number of counts the ADC can have
    }
    
    namespace torque_calibration
    {
        const float AI_CNT_TO_V = 3.3 / 4096;   //Conversion from count to voltage
        const float TRQ_V_TO_NM = 53.70;        //Conversion from voltage to Nm (Negative do to mismatch in torque sensor and motor torque directions) S12:(Left) = 39.8, S05 (Right) = 44.6; (These will be sensor specific).
    }

    namespace BLE_times
    {
        const float _status_msg_delay = 2000000;    //Microseconds
        const float _real_time_msg_delay = 9000;    //Microseconds (~111 Hz target, expect ~100 Hz actual)
        const float _update_delay = 1000;           //Microseconds
        const float _poll_timeout = 4;              //Milliseconds
    }
    
    //Update this namespace for future exo updates to display correct information on app
    namespace exo_info
    {
        const String FirmwareVersion = String(TOSTRING(FIRMWARE_VERSION));  //String to add to firmware char
        const String PCBVersion = String(TOSTRING(BOARD_VERSION));          //String to add to pcb char
        const String DeviceName = String("NULL");                            //String to add to device char, if you would like the system to set it use "NULL"
    }

    namespace UART_times
    {
        const float UPDATE_PERIOD = 1000;       //Microseconds, time between updating data over uart
        const float COMS_MCU_TIMEOUT = 5000;    //Microseconds
        const float CONT_MCU_TIMEOUT = 1000;    //Microseconds
        const float CONFIG_TIMEOUT = 8000;      //Milliseconds
    }

#endif
