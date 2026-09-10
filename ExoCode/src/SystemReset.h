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

/* ============================ CRASH TRAP (Nano) ============================
 *
 * WHY: RESETREAS above answers "why did the Nano reset". It CANNOT answer the question we actually
 * have, because when the Nano dies mid-trial it does not reset at all - it stops dead, the radio
 * goes quiet, BLE times out ~9.6 s later, and the only way back is a hand power cycle. A power-on
 * reads RESETPIN on this board (measured 2026-09-09), so by the time anyone connects, every trace of
 * the original fault is gone. Nothing in RAM survives either - not ErrorChar, not GPREGRET.
 *
 * WHAT THIS DOES: mbed routes hard faults, failed asserts and allocation failures through
 * mbed_error(), whose default behaviour is to HALT FOREVER with interrupts off - which is exactly the
 * dead-radio symptom. Overriding mbed_error_hook() lets us instead stash a compact description of the
 * fault in GPREGRET/GPREGRET2 and reboot. Two wins: the Nano comes back on its own in ~2 s instead of
 * needing a power cycle, and the next boot can tell the GUI what killed it.
 *
 * WHY GPREGRET: it is retained across a warm reset (which ours is) but cleared by a true power-on.
 * That is precisely the distinction we need, and it is the reason this works where RESETREAS does not.
 *
 * BOOT-LOOP GUARD: a fault that reproduces immediately would otherwise reset forever. After
 * EXO_CRASH_MAX_AUTO_RESETS consecutive crashes the hook returns instead, letting mbed halt as it
 * normally would, so a wedged device stays wedged and visible rather than thrashing. The count is
 * cleared by exo_crash_mark_healthy(), called once the GUI has actually subscribed.
 */

#if defined(EXO_HAVE_NRF_RESETREAS)
#  include "platform/mbed_error.h"
#  define EXO_HAVE_CRASH_TRAP 1
#endif

#define EXO_CRASH_MAGIC             0xC0u   //High nibble marks "GPREGRET holds a crash record"
#define EXO_CRASH_MAGIC_MASK        0xF0u
#define EXO_CRASH_COUNT_MASK        0x0Fu
#define EXO_CRASH_MAX_AUTO_RESETS   3

/**
 * @brief Latched GPREGRET pair from the previous boot: {marker byte, error byte}. Captured once.
 *
 * Same latch-then-clear discipline as exo_reset_reason_code(). We write back magic + count only, so
 * the consecutive-crash count survives into the next boot while the error byte starts clean.
 */
inline void exo_crash_record(uint8_t* marker_out, uint8_t* error_out)
{
#if defined(EXO_HAVE_CRASH_TRAP)
    static bool captured = false;
    static uint8_t latched_marker = 0;
    static uint8_t latched_error = 0;
    if (!captured)
    {
        captured = true;
        latched_marker = (uint8_t)NRF_POWER->GPREGRET;
        latched_error  = (uint8_t)NRF_POWER->GPREGRET2;
        if ((latched_marker & EXO_CRASH_MAGIC_MASK) == EXO_CRASH_MAGIC)
        {
            //Keep the count, drop the error byte - the count is what the boot-loop guard needs
            NRF_POWER->GPREGRET  = (uint32_t)(latched_marker & (EXO_CRASH_MAGIC_MASK | EXO_CRASH_COUNT_MASK));
            NRF_POWER->GPREGRET2 = 0;
        }
        else
        {
            latched_marker = 0;   //Not ours (a true power-on clears these), so report nothing
            latched_error = 0;
            NRF_POWER->GPREGRET  = 0;
            NRF_POWER->GPREGRET2 = 0;
        }
    }
    if (marker_out) { *marker_out = latched_marker; }
    if (error_out)  { *error_out  = latched_error; }
#else
    if (marker_out) { *marker_out = 0; }
    if (error_out)  { *error_out  = 0; }
#endif
}

#define EXO_CRASH_TRAP_SELFTEST 0   //1 -> deliberately hard-fault ONCE at boot to prove the trap works.

/**
 * @brief Prove the crash trap end to end. Does nothing unless EXO_CRASH_TRAP_SELFTEST is 1.
 *
 * WHY THIS IS NEEDED: a silent trap and a broken trap look identical. Until something has actually
 * faulted, "no CRASH line in the banner" could mean either "nothing has crashed" or "mbed never
 * reaches our hook on this core and never will". This settles which.
 *
 * It provokes a REAL bus fault (a write to a reserved address) rather than calling mbed_error()
 * directly, so it exercises the whole chain we care about: hardware fault -> mbed fault handler ->
 * mbed_error() -> mbed_error_hook() -> GPREGRET -> reboot -> banner.
 *
 * It fires only when the crash count is 0, so it crashes exactly ONCE per fresh power-on: boot,
 * fault, reboot, and the second boot comes up normally carrying the record. Connect and you should
 * see "*** THE NANO CRASHED AND REBOOTED ITSELF ***". Then set this back to 0 and reflash.
 *
 * Safe to run: this happens during ExoBLE::setup(), long before any trial, with motors unpowered.
 */
inline void exo_crash_trap_selftest()
{
#if defined(EXO_HAVE_CRASH_TRAP) && (EXO_CRASH_TRAP_SELFTEST == 1)
    uint8_t marker = 0;
    uint8_t err = 0;
    exo_crash_record(&marker, &err);   //Latches, so this reads the PREVIOUS boot's record
    if ((marker & EXO_CRASH_MAGIC_MASK) != EXO_CRASH_MAGIC)
    {
        //No crash recorded, so this is a clean boot: fault now, once.
        volatile uint32_t* bad = (volatile uint32_t*)0xFFFFFFF0u;
        *bad = 0xDEADBEEFu;
    }
#endif
}

/**
 * @brief Clear the consecutive-crash count. Call once the link is genuinely up and serving.
 *
 * Without this the count only ever grows and the boot-loop guard would eventually stop recovering
 * from unrelated, widely-spaced faults.
 */
inline void exo_crash_mark_healthy()
{
#if defined(EXO_HAVE_CRASH_TRAP)
    NRF_POWER->GPREGRET = 0;
#endif
}

//The hook itself lives in SystemReset.cpp. It MUST be in exactly one translation unit: this header
//is included from several .cpp files, and a non-inline definition here produced
//"multiple definition of mbed_error_hook" at link time.

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

    //If the previous boot ended in a trapped fault, append it to the NAMES field rather than adding
    //a field of its own. The GUI splits this string on ':' and prints everything in names verbatim
    //(MainWindow._on_reset_reason), so this shows up with no GUI change at all. ErrorChar is 255
    //bytes and variable length, so the extra characters fit comfortably.
    uint8_t crash_marker = 0;
    uint8_t crash_error = 0;
    exo_crash_record(&crash_marker, &crash_error);
    if ((crash_marker & EXO_CRASH_MAGIC_MASK) == EXO_CRASH_MAGIC)
    {
        char crash[40];
        snprintf(crash, sizeof(crash), ",CRASH_0x%02X_n%u",
                 (unsigned)crash_error, (unsigned)(crash_marker & EXO_CRASH_COUNT_MASK));
        return String(head) + names + String(crash);
    }

    return String(head) + names;
}

#endif
