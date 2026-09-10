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

//Shared by the crash trap and the watchdog boot-loop guard below - both pack a magic in the high
//nibble of GPREGRET and a consecutive-reset count in the low nibble.
#define EXO_CRASH_MAGIC             0xC0u   //High nibble marks "GPREGRET holds a crash record"
#define EXO_CRASH_MAGIC_MASK        0xF0u
#define EXO_CRASH_COUNT_MASK        0x0Fu
#define EXO_CRASH_MAX_AUTO_RESETS   3

//Forward declaration: exo_wdt_boot_count() below must latch any pending crash record BEFORE it
//overwrites GPREGRET with the watchdog count. Defined further down; inline, same translation unit.
inline void exo_crash_record(uint8_t* marker_out, uint8_t* error_out);

/* ===========================  HARDWARE WATCHDOG  ===========================
 * WHY THIS EXISTS: the mid-trial freeze is a HANG, not a fault. Proven
 * 2026-09-10 - after a freeze the user pressed the Nano's RESET BUTTON instead
 * of power-cycling (a pin reset is warm, so RESETREAS survives) and the reading
 * came back 0x00000001, RESETPIN alone. The firmware zeroes RESETREAS every
 * boot, so any reset during the freeze would have left its bit set: no LOCKUP
 * (no double fault), no SREQ (so mbed_error_hook's NVIC_SystemReset never ran),
 * no DOG, and not 0x0/PORBOR (no brownout, no power-on). Nothing reset the MCU.
 * It stopped and stayed stopped. Independently confirmed by the onboard LED:
 * ComsLed::life_pulse() toggles green+blue every 100 loop passes, so a live
 * Nano's LED never shows a steady colour - it blends. At the freeze it froze on
 * one ENDPOINT of the toggle (white one time, red the next - random phase),
 * which means local_sample() stopped being called, which means loop() stopped.
 *
 * mbed_error_hook CANNOT catch this: a wedged loop raises no fault, so nothing
 * is ever written to GPREGRET and nothing reboots. Only a hardware timer that
 * the CPU must keep feeding can.
 *
 * WHY IT NEED NOT KNOW WHERE THE BUG IS: the WDT counts off the 32.768 kHz
 * LFCLK, independently of the CPU. We feed it once per loop() pass. If anything,
 * anywhere, stops loop() coming round, it fires - no knowledge of the fault site
 * required. That is exactly why it works where every software timeout did not.
 *
 * WHY loop() AND NOT A CALLBACK: ArduinoBLE runs Cordio in its own RTOS thread,
 * which is why a stalled loop() alone would NOT drop the BLE link. Feeding from
 * anything that survives the hang would defeat the whole point. loop() is the
 * thing we proved stops.
 *
 * WHY IT ARMS LATE: boot spends up to 18 s in readSingleMessageBlocking()
 * (10 s kReadyTimeoutMs + 8 s kReceiveTimeoutMs) and up to 8 s in get_config(),
 * with nothing feeding. Arming before that would reset the board mid-boot,
 * forever. exo_wdt_start() is called at the END of setup(), once loop() is
 * about to run.
 *
 * ONCE STARTED, THE nRF52840 WDT CANNOT BE STOPPED. By design - and the reason
 * the arming point above matters.
 *
 * WHY 5 SECONDS: comfortably under the ~9.6 s BLE supervision timeout, so the
 * Nano is already rebooting and re-advertising before the GUI even notices the
 * link is gone.
 *
 * THE BREADCRUMB: the WDT recovers the board but says nothing about WHERE it
 * hung. exo_wdt_stage() stamps a one-byte stage code into GPREGRET2 as loop()
 * passes each phase. GPREGRET2 is retained across a warm reset - and a watchdog
 * reset IS warm - so the next boot reads back the last stage reached before the
 * hang. "It hung somewhere" becomes "it hung between stage N and stage N+1".
 *
 * GPREGRET2 IS SHARED with the crash trap below, which stores an mbed error byte
 * there. They are told apart by GPREGRET: if it holds the crash magic the byte
 * is an error code, otherwise it is a stage breadcrumb.
 */
#if defined(EXO_HAVE_NRF_RESETREAS)
#  define EXO_HAVE_WDT 1
#endif

//Watchdog boot-loop guard. Shares GPREGRET with the crash trap below, distinguished by magic:
//0xC0 = "a trapped mbed fault happened", 0xD0 = "consecutive watchdog reboots". Both are cleared by
//exo_crash_mark_healthy() once the GUI actually subscribes, which is the definition of "recovered".
#define EXO_WDT_MAGIC             0xD0u
#define EXO_WDT_MAX_AUTO_RESETS   3u

#define EXO_WDT_TIMEOUT_S       5u
#define EXO_WDT_RELOAD_MAGIC    0x6E524635ul   //nRF52840 WDT reload key, fixed by the datasheet
#define EXO_WDT_LFCLK_HZ        32768ul

//loop() phase codes, stamped into GPREGRET2 by exo_wdt_stage(). Values are arbitrary but must be
//non-zero: 0 means "no breadcrumb recorded", which is what a boot with no prior stage looks like.
#define EXO_STAGE_LOOP_TOP      1u
#define EXO_STAGE_HANDLE_BLE    2u
#define EXO_STAGE_LOCAL_SAMPLE  3u
#define EXO_STAGE_UPDATE_UART   4u
#define EXO_STAGE_UPDATE_GUI    5u
#define EXO_STAGE_HANDLE_ERRORS 6u

/**
 * @brief Stamp the current loop() phase into GPREGRET2. Cheap: one register write, no branch.
 */
inline void exo_wdt_stage(uint8_t stage)
{
#if defined(EXO_HAVE_WDT)
    NRF_POWER->GPREGRET2 = (uint32_t)stage;
#else
    (void)stage;
#endif
}

/**
 * @brief Latch the PREVIOUS boot's stage breadcrumb, once, before anything clears GPREGRET2.
 *
 * MUST be called before exo_crash_record(), which is the only thing that clears GPREGRET2.
 * exo_crash_record() calls this itself as its first action, so ordering is guaranteed no matter
 * which one a caller reaches first.
 */
inline uint8_t exo_wdt_stage_record()
{
#if defined(EXO_HAVE_WDT)
    static bool captured = false;
    static uint8_t latched_stage = 0;
    if (!captured)
    {
        captured = true;
        latched_stage = (uint8_t)NRF_POWER->GPREGRET2;
    }
    return latched_stage;
#else
    return 0;
#endif
}

/**
 * @brief Feed the watchdog. Call once per loop() pass, from loop() itself.
 */
inline void exo_wdt_feed()
{
#if defined(EXO_HAVE_WDT)
    NRF_WDT->RR[0] = EXO_WDT_RELOAD_MAGIC;
#endif
}

/**
 * @brief Consecutive watchdog reboots, counted and stored on each DOG boot. 0 on any other reset.
 *
 * WHY: without this a Nano that hangs immediately on every boot would reboot forever, and - worse -
 * it would defeat the crash trap's own guard below, whose whole point is to let a reproducible fault
 * leave the device wedged-but-visible instead of thrashing. A dog would just reboot the halted mbed.
 *
 * Uses exo_reset_reason_code() rather than reading RESETREAS directly so both share one latch. That
 * accessor clears the hardware register on first call; calling it here (end of setup) simply latches
 * earlier than ExoBLE::setup() would, and every later reader gets the same value.
 */
inline uint8_t exo_wdt_boot_count()
{
#if defined(EXO_HAVE_WDT)
    static bool counted = false;
    static uint8_t count = 0;
    if (!counted)
    {
        counted = true;
        const uint32_t reasons = exo_reset_reason_code();
        if ((reasons != 0xFFFFFFFFu) && (reasons & 0x00000002ul))   //DOG
        {
            //A fault -> mbed halt -> dog reboot leaves a CRASH record sitting in GPREGRET that has
            //not been read yet (exo_crash_record runs later, from ExoBLE::setup()). Latch it now,
            //before the write below replaces it, or that reboot silently eats the crash code.
            //The C-count is lost in that corner case, which is fine: the dog guard below takes over
            //bounding the loop, and exo_crash_mark_healthy() clears both on a successful connect.
            exo_crash_record(0, 0);

            const uint8_t marker = (uint8_t)NRF_POWER->GPREGRET;
            const uint8_t prev = ((marker & EXO_CRASH_MAGIC_MASK) == EXO_WDT_MAGIC)
                                 ? (uint8_t)(marker & EXO_CRASH_COUNT_MASK) : 0u;
            count = (uint8_t)((prev + 1u) & EXO_CRASH_COUNT_MASK);
            NRF_POWER->GPREGRET = (uint32_t)(EXO_WDT_MAGIC | count);
        }
    }
    return count;
#else
    return 0;
#endif
}

/**
 * @brief Configure and start the watchdog. Call at the END of setup(). Cannot be undone.
 */
inline void exo_wdt_start(uint32_t timeout_s = EXO_WDT_TIMEOUT_S)
{
#if defined(EXO_HAVE_WDT)
    //Latch the PREVIOUS boot's breadcrumb here, at the end of setup(), because loop() starts
    //overwriting GPREGRET2 with THIS boot's stages on its very first pass. Doing it here rather than
    //relying on exo_crash_record() being reached first removes the ordering dependency entirely.
    (void)exo_wdt_stage_record();

    if (NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk)
    {
        return;   //Already running; starting twice is harmless but pointless
    }

    //Boot-loop guard: after this many consecutive watchdog reboots, stop arming. The device then
    //stays up, advertising and connectable, so the STAGE breadcrumb can actually be READ instead of
    //being rebooted away every 5 s. Deliberately leaves a hung exo hung - that is the safer failure.
    if (exo_wdt_boot_count() >= EXO_WDT_MAX_AUTO_RESETS)
    {
        return;
    }

    //SLEEP=1: keep counting while the CPU sleeps. mbed idles the core between events, so without
    //this a hang that parks in sleep would never trip the dog - which is most of them.
    //HALT=0: pause while halted by a debugger, so single-stepping does not reset the board.
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
                      (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);

    NRF_WDT->CRV  = (timeout_s * EXO_WDT_LFCLK_HZ) - 1ul;
    NRF_WDT->RREN = (WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos);   //Only reload register 0 is armed

    exo_wdt_feed();                 //Start from a full counter
    NRF_WDT->TASKS_START = 1ul;
#else
    (void)timeout_s;
#endif
}


/**
 * @brief Latched GPREGRET pair from the previous boot: {marker byte, error byte}. Captured once.
 *
 * Same latch-then-clear discipline as exo_reset_reason_code(). We write back magic + count only, so
 * the consecutive-crash count survives into the next boot while the error byte starts clean.
 */
inline void exo_crash_record(uint8_t* marker_out, uint8_t* error_out)
{
#if defined(EXO_HAVE_CRASH_TRAP)
    //FIRST: this function is the only thing that clears GPREGRET2, and on a watchdog reset that
    //register holds the loop() stage breadcrumb. Latch it before we touch anything.
    (void)exo_wdt_stage_record();

    static bool captured = false;
    static uint8_t latched_marker = 0;
    static uint8_t latched_error = 0;
    if (!captured)
    {
        captured = true;
        latched_marker = (uint8_t)NRF_POWER->GPREGRET;
        latched_error  = (uint8_t)NRF_POWER->GPREGRET2;
        const uint8_t latched_marker_raw = latched_marker;
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
            //Only clear GPREGRET when it is NOT holding the watchdog reboot count - zeroing that
            //here would silently disarm the boot-loop guard, since this runs on every boot.
            if ((latched_marker_raw & EXO_CRASH_MAGIC_MASK) != EXO_WDT_MAGIC)
            {
                NRF_POWER->GPREGRET = 0;
            }
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

    //A watchdog reset means the previous boot HUNG rather than faulted, so GPREGRET2 holds a loop()
    //stage breadcrumb instead of an mbed error byte (the crash magic above is what tells them
    //apart). Appended into names the same way CRASH_ is, so the GUI prints it with no change.
    if (reasons & 0x00000002ul)
    {
        char stage[24];
        snprintf(stage, sizeof(stage), ",STAGE_%u_dog%u",
                 (unsigned)exo_wdt_stage_record(), (unsigned)exo_wdt_boot_count());
        return String(head) + names + String(stage);
    }

    return String(head) + names;
}

#endif
