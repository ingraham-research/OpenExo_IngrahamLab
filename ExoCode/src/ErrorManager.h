/**
 * @file ErrorManager.h
 * @author Chancelor Cuddeback
 * @brief Checks for errors and runs error handlers. Places errors in a queue.
 * 
 */

#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "error_types.h"
#include "error_codes.h"
#include "error_map.h"
#include "Config.h"     //ERROR_MANAGER_ENABLED
#include <queue>

//Fail closed: if Config.h somehow is not in scope, stay disabled rather than silently re-enabling.
#ifndef ERROR_MANAGER_ENABLED
    #define ERROR_MANAGER_ENABLED 0
#endif

/**
 * @brief Class manages the calling of error handlers and triggers. Only the control MCU is required to use this.
 * The triggers and handlers must be assigned before check is ran. They are defined in error_triggers.h and error_handlers.h.
 * Check should be called every loop, and should not, severely, increase the loop time. 
 */
class ErrorManager
{
    public:
        ErrorManager() {};
        
        /**
         * @brief Runs the error manager. This should be called every loop.
         * 
         * @return true No errors
         * @return false 
         */
        template <typename data>
        bool run(data* _data)
        {
        #if !ERROR_MANAGER_ENABLED
            // FEATURE TEMPORARILY DISABLED - see ERROR_MANAGER_ENABLED in Config.h for the full
            // reasoning and the checklist that must be satisfied before turning it back on.
            //
            // Short version: no handler takes any protective action, nothing downstream receives
            // the errors, and the checks cannot fire anyway (five are hardcoded false, MotorTimeout
            // is unreachable, and TorqueVarianceError's 10-sigma test has a mathematical ceiling of
            // 9.9 sigma). So the per-cycle cost - two full copies of a 100-element std::queue plus a
            // 100-iteration Welford pass, per joint - buys a conclusion that can never change.
            // Returning false here compiles out all eight checks, so the queue stays empty and
            // Joint.cpp's `if (error)` block never runs.
            //
            // Safe to short-circuit: every field these checks touch (smoothed_motor_torque,
            // torque_error, torque_data_window, torque_failure_count) is written and read ONLY
            // inside the check that owns it - verified, no external consumer. motor.timeout_count
            // is only ever set to 0 elsewhere, so nothing depends on MotorTimeoutError clearing it.
            (void)_data;
            return false;
        #else
            //Check for errors
            for (int i_error = (NO_ERROR + 1); i_error != ERROR_CODE_LENGTH; i_error++)
            {
                //Current error code
                ErrorCodes error_code = static_cast<ErrorCodes>(i_error);

                //Get error type
                ErrorType* error = error_map.at(error_code);

                //Check for error
                if (error->check(_data))
                {
                    error->handle(_data);
                    this->_pushError(error_code);
                }
            }

            return static_cast<bool>(this->errorQueueSize());
        #endif
        }

        /**
         * @brief Check the size of the error queue
         * 
         * @return int 
         */
        int errorQueueSize()
        {
            return this->_error_queue.size();
        }

        /**
         * @brief Get the next error code in the queue
         * 
         * @return ErrorCodes 
         */
        ErrorCodes popError()
        {
            ErrorCodes error_code = this->_error_queue.front();
            this->_error_queue.pop();
            return error_code;
        }

    private:
        std::queue<ErrorCodes> _error_queue;
        
        /**
         * @brief Pushes an error code to the queue
         * 
         * @param error_code 
         */
        void _pushError(ErrorCodes error_code)
        {
            this->_error_queue.push(error_code);
        }

};

#endif
#endif // ERROR_MANAGER_H