/**
 * @file SystemReset.cpp
 * @brief The mbed fatal-error hook. Definition only - see SystemReset.h for the reasoning.
 *
 * This exists as its own translation unit because SystemReset.h is included from several .cpp
 * files, and mbed_error_hook is a real (non-inline) function: defining it in the header gives
 * "multiple definition of mbed_error_hook" at link. Nothing here is called by our code - mbed
 * calls it from mbed_error() when it is about to halt.
 *
 * Compiles to nothing on the Teensy, which has neither mbed nor GPREGRET.
 */
#include "SystemReset.h"

#if defined(EXO_HAVE_CRASH_TRAP)

/**
 * @brief mbed's fatal-error hook. Runs in fault context: no printing, no allocation, no blocking.
 *
 * mbed declares this weak and calls it from mbed_error() before it halts. We record and reboot.
 */
extern "C" void mbed_error_hook(const mbed_error_ctx *error_context)
{
    uint8_t marker = (uint8_t)NRF_POWER->GPREGRET;
    uint8_t count = ((marker & EXO_CRASH_MAGIC_MASK) == EXO_CRASH_MAGIC)
                    ? (uint8_t)(marker & EXO_CRASH_COUNT_MASK) : 0u;

    if (count >= EXO_CRASH_MAX_AUTO_RESETS)
    {
        return;   //Give up and let mbed halt, rather than boot-looping on a fault that reproduces
    }

    uint8_t err_byte = 0;
    if (error_context != 0)
    {
        //Low 8 bits of the mbed error CODE. Enough to name the fault class (hard fault, assert,
        //out-of-memory, stack overflow) without needing more retained registers than we have.
        err_byte = (uint8_t)(MBED_GET_ERROR_CODE(error_context->error_status) & 0xFFu);
    }

    NRF_POWER->GPREGRET  = (uint32_t)(EXO_CRASH_MAGIC | ((count + 1u) & EXO_CRASH_COUNT_MASK));
    NRF_POWER->GPREGRET2 = (uint32_t)err_byte;

    NVIC_SystemReset();   //Includes the DSB that flushes the two writes above
}
#endif
