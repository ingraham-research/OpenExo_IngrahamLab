/**
 * @file ErrorReporter.h
 * @author Chancelor Cuddeback
 * @brief Singleton class to report errors to the other microcontroller.
 * 
 */

#ifndef ERROR_REPORTER_H
#define ERROR_REPORTER_H

#include "UARTHandler.h"
#include "uart_commands.h"
#include "error_codes.h"
#include "ParseIni.h"
#include "UART_msg_t.h"
#include "Config.h"     //ERROR_MANAGER_ENABLED

//Fail closed: if Config.h somehow is not in scope, stay disabled rather than silently re-enabling.
#ifndef ERROR_MANAGER_ENABLED
    #define ERROR_MANAGER_ENABLED 0
#endif

/**
 * @brief Singleton class to report errors to the other microcontroller.
 * 
 */
class ErrorReporter
{
    ErrorReporter() {};
    ~ErrorReporter() {};
public:
    /**
     * @brief Get the instance object
     * 
     * @return ErrorReporter* 
     */
    static ErrorReporter* get_instance()
    {
        static ErrorReporter* instance = new ErrorReporter();
        return instance;
    }

    /**
     * @brief Report an error to the other microcontroller.
     * 
     * @param error_code 
     * @param joint_id 
     */
    void report(ErrorCodes error_code, config_defs::joint_id joint_id)
    {
    #if !ERROR_MANAGER_ENABLED
        // FEATURE TEMPORARILY DISABLED - see ERROR_MANAGER_ENABLED in Config.h.
        //
        // Second gate, belt-and-braces. ErrorManager::run() already returns false so Joint.cpp
        // never reaches this call, but this makes the whole feature dead from a single switch even
        // if a new caller appears. It is deliberately silent: adding a print here would put a
        // blocking Serial write in the control loop, which is the class of problem being removed.
        //
        // Why this gate matters more than it looks: UARTHandler::UART_msg() ends in
        // MY_SERIAL.flush(), which SPINS until the UART shift register drains (~234 us per 6-byte
        // SLIP frame at 256000 baud 8N1). No caller reaches it today, but a per-control-cycle
        // caller would cost ~23% of the loop across two ankles and would exceed what
        // ComsMCU::update_UART can consume (one message per 1000 us). See Config.h.
        (void)error_code;
        (void)joint_id;
        return;
    #else
        UART_msg_t msg;
        msg.joint_id = static_cast<uint8_t>(joint_id);
        msg.data[(uint8_t)UART_command_enums::get_error_code::ERROR_CODE] = (float)static_cast<int>(error_code);
        UART_command_handlers::get_error_code(
            UARTHandler::get_instance(), nullptr, msg);
    #endif
    }
};

#endif