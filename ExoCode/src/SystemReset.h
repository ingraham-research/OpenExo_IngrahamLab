/**
 * @file SystemReset.h
 * @brief Helpers for triggering a MCU reset across supported boards.
 */
#ifndef SYSTEM_RESET_H
#define SYSTEM_RESET_H

#include "Arduino.h"
#include <stdio.h>   //snprintf, used to format the reset-reason string

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)
#elif defined(ARDUINO_ARDUINO_NANO33BLE) || defined(ARDUINO_NANO_RP2040_CONNECT)

//nRF52840 RESETREAS lives behind the nRF CMSIS header. __has_include keeps this file compiling
//on any core that does not ship it; the reason readout then reports UNAVAILABLE instead of
//breaking the build.
//
//This include MUST come before the NVIC_SystemReset declaration below. CMSIS core_cm4.h defines
//NVIC_SystemReset as a MACRO aliasing a __STATIC_INLINE function, so declaring it ourselves after
//that macro is visible would expand to a non-static redeclaration of a static inline - a compile
//error. Including first and then guarding on the macro keeps both orders safe.
#if defined(__has_include)
#  if __has_include("nrf.h")
#    include "nrf.h"
#    define EXO_HAVE_NRF_RESETREAS 1
#  endif
#endif

#ifndef NVIC_SystemReset
extern "C" void NVIC_SystemReset(void);
#endif
#endif

inline void exo_system_reset()
{
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    CPU_RESTART;
#elif defined(ARDUINO_ARDUINO_NANO33BLE) || defined(ARDUINO_NANO_RP2040_CONNECT)
    NVIC_SystemReset();
#endif
}

/**
 * @brief Raw RESETREAS bits latched from the last reset, captured once on first call.
 *
 * WHY THIS EXISTS: the GUI's mid-trial "unexpectedly disconnected" is a BLE link supervision
 * timeout - the Nano stops transmitting entirely and Windows tears the link down ~9.6 s later.
 * The Teensy is provably still alive at that instant (its millis() clock, streamed as
 * "Exoskeleton time", advances to the final delivered sample), so the question is what happens
 * to the Nano. RESETREAS answers it directly:
 *
 *   0x0 (no bits)  power-on or BROWNOUT   -> a power event, or someone power-cycled it
 *   RESETPIN       reset pin / double-tap -> reset button, or a re-flash
 *   DOG            watchdog               -> this firmware configures none, so unexpected
 *   SREQ           NVIC_SystemReset()     -> our End-Trial 'Z' path, or an Mbed fault auto-reboot
 *   LOCKUP         CPU lockup             -> hard fault escalated: a firmware crash
 *
 * RESETREAS is CUMULATIVE - bits stay set across resets until written back - so this latches the
 * value once and clears it, leaving the register clean for the next reset to describe itself.
 *
 * NOTE: this reports why the Nano last *reset*. If the Nano instead HANGS with its radio dead and
 * has to be power-cycled by hand, that reads back as power-on (0x0), which is indistinguishable
 * here from a brownout. Pair it with whether the exo re-advertised on its own to tell those apart.
 */
inline uint32_t exo_reset_reason_code()
{
#if defined(EXO_HAVE_NRF_RESETREAS)
    static bool captured = false;
    static uint32_t latched = 0;
    if (!captured)
    {
        captured = true;
        latched = NRF_POWER->RESETREAS;
        NRF_POWER->RESETREAS = latched;   //write-1-to-clear, so the next reset starts clean
    }
    return latched;
#else
    return 0xFFFFFFFFu;   //sentinel: register not reachable on this core
#endif
}

/**
 * @brief Human-readable reset reason, formatted for the GUI: "RST:0x<hex>:<names>".
 *
 * The "RST:" prefix is what lets the GUI tell this apart from a runtime error report, which
 * shares the same characteristic and uses the "<code>:<joint>" format (see ExoBLE::send_error).
 */
inline String exo_reset_reason_string()
{
    const uint32_t reasons = exo_reset_reason_code();

    if (reasons == 0xFFFFFFFFu)
    {
        return String("RST:UNAVAILABLE");
    }

    String names = "";
    if (reasons & 0x00000001ul) { names += "RESETPIN,"; }
    if (reasons & 0x00000002ul) { names += "DOG,"; }
    if (reasons & 0x00000004ul) { names += "SREQ,"; }
    if (reasons & 0x00000008ul) { names += "LOCKUP,"; }
    if (reasons & 0x00010000ul) { names += "OFF,"; }
    if (reasons & 0x00020000ul) { names += "LPCOMP,"; }
    if (reasons & 0x00040000ul) { names += "DIF,"; }
    if (reasons & 0x00080000ul) { names += "NFC,"; }
    if (reasons & 0x00100000ul) { names += "VBUS,"; }

    if (names.length() == 0)
    {
        //No bit set is meaningful, not missing data: POR/BOR clears the whole register.
        names = "PORBOR";
    }
    else
    {
        names.remove(names.length() - 1);   //trailing comma
    }

    //snprintf rather than String concatenation on purpose: `"0" + hex` (const char* on the LEFT of
    //a String) has no operator overload in Arduino's WString.h and does not compile. Formatting the
    //fixed-width hex here sidesteps that entirely.
    char head[20];
    snprintf(head, sizeof(head), "RST:0x%08lX:", (unsigned long)reasons);

    return String(head) + names;
}

#endif
