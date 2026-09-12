/**
 * @file SystemReset.h
 * @brief Helpers for triggering a MCU reset across supported boards.
 */
#ifndef SYSTEM_RESET_H
#define SYSTEM_RESET_H

#include "Arduino.h"
#include <stdio.h>   //snprintf, used to format the reset-reason string
#include "Config.h"  //REAL_TIME_I2C, reported in the banner as rt<0|1>

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
inline uint8_t exo_stall_record();

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
//0xE0 = "the BLE link went dead while we still thought we were connected, so we reset ourselves".
//This is the ONLY marker that positively confirms the link-stall diagnosis: it is written by our own
//code on a deliberate warm reset, so seeing it in the banner means the detector fired, not a guess.
//Build tag, appended to every reset-reason banner. Bump this whenever the instrumentation changes.
//WHY: three bench runs were spent arguing about what a STAGE_0 meant, when the real question was
//"which firmware is actually on the board". The banner now answers that itself.
//  9 = boot-path stages 8-15, GPREGRET2 no longer blanked, send-path diag with the 0x80 marker
// 10 = .noinit RAM breadcrumb alongside GPREGRET2, banner reports both as STAGEn<n>_g<g>
// 11 = .noinit breadcrumb latched at the top of setup() with the register copy. B10 CONFIRMED
//      that a watchdog reset wipes GPREGRET2 (g0 with a known non-zero stage in flight), so
//      the RAM copy is the one to trust - B10's STAGEn15 was it reporting its own footprint.
// 12 = Teensy RT-I2C health counters fetched at boot and appended as I2Cf<n>_e<n>_c<n>_t<n>
// 13 = adds w<n>, the WORST gap ever seen between successful RT transmissions. t is sampled
//      after the Nano's ~18-26 s reboot and so describes the recovery, not the failure; w does
//      not move once the event is over, so it is the field to trust.
// 14 = link stats pre-scaled + clamped for the int16x100 UART (B13's f0/e165 was a wrap), and
//      a .noinit boot counter reported as b<n> to settle whether that section is retained
// 15 = B14 ANSWERED THAT: b1 on every boot, so .noinit is zeroed at startup and neither it nor
//      GPREGRET2 survives a watchdog reset - the STAGE breadcrumb has never had a working
//      store, do not trust STAGEn/g. Adds x<n> = longest run of consecutive I2C failures, and
//      fixes worst_gap measuring its first interval from boot instead of from a real success.
// 16 = RT-I2C BISECT BUILD. REAL_TIME_I2C forced to 0 in Config.h; banner carries rt<0|1> so the
//      build is never in doubt. B15 established the failure is an interrupt-level stop, not a
//      loop hang (x300 capped = >=3 s of unbroken I2C silence, both LEDs frozen), and no
//      in-Nano store survives it - so this bisects by subsystem instead of instrumenting more.
// 17 = bisect round 2. REAL_TIME_I2C back to 1, RT_BLE_FORWARD 0: I2C runs normally, only the
//      BLE forward is suppressed. Round 1 (rt0) survived ~40 min but removed I2C traffic AND
//      the BLE notification flood together; this splits them. Banner carries rt<n>_fw<n>.
// 18 = NORMAL OPERATION restored (rt1_fw1) PLUS the bounded-wait patch to the sketchbook copy of
//      ArduinoBLE (HCI.cpp sendAclPkt - see that file comment and doc section 8). THIS BUILD IS THE
//      TEST OF THE FIX: if presentation B (everything frozen solid) stops happening, the unbounded
//      spin was the amplifier. If it still happens, the spin is exonerated - cheaply either way.
// 19 = B18 CONFIRMED the spin was the amplifier (w10 = the 50 ms timeout firing; DOG became
//      BLESTALL; RGB fixed while green kept flashing = a live loop slowed to ~20 Hz). But the
//      link still dies as often - two runs at 41 s and 24 s. So B19 is the FIRST change aimed at
//      WHY it dies: connection interval 7.5 ms rigid -> 15-30 ms range (ExoBLE.cpp).
// 20 = A/B/A CONTROL, arm A' (doc section 14.3). Connection interval deliberately put BACK to
//      (6,6) = 7.5 ms pinned - the configuration that failed 10/10 - to test whether the failure
//      returns. Section 9 rests on a BETWEEN-GROUPS comparison (10 failures on the old build, 2
//      clean runs on the new one) separated by several days, a library patch, a ping, a lock and a
//      host reboot. Nothing has ever gone BACK. This build makes the interval the only moving part.
//      EXPECTED: failure inside ~4 min (baseline mean 225 s). If it does NOT fail, section 9 is in
//      serious trouble and something else was the real fix.
// 21 = A/B/A, arm B again: interval restored to (12,24). Flip EXO_BLE_INTERVAL_PINNED to 0.
#define EXO_BLE_INTERVAL_PINNED   0u
//        1 = (6, 6)   7.5 ms PINNED  - the original, known-failing configuration   -> build B20
//        0 = (12, 24) 15-30 ms range - the candidate fix from section 9            -> build B21
//
// The build tag is DERIVED from that toggle on purpose. Flipping the interval and forgetting to
// bump the tag would make the banner claim the wrong build, and the whole point of an A/B/A is that
// every log is unambiguously attributable to one arm. They cannot disagree now.
#define EXO_FW_TAG                (EXO_BLE_INTERVAL_PINNED ? 20u : 21u)

#define EXO_STALL_MAGIC           0xE0u

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
//Stamped by exo_wdt_start(). Seeing this in a banner means the dog fired after arming but before the
//loop reached its first breadcrumb - i.e. a boot-path stall, not a steady-state hang. STAGE_0 now
//means only "the previous boot recorded nothing", e.g. it ended in a true power-on.
#define EXO_STAGE_ARMED         7u

//BOOT-PATH stages. Added after three runs reported the useless STAGE_0: the loop stages only cover
//steady state, so anything dying before the first complete pass was indistinguishable from "nothing
//recorded". These are stamped from setup() onward so the breadcrumb is NEVER 0 on a live board.
#define EXO_STAGE_SETUP_ENTRY   8u   //setup() reached at all
#define EXO_STAGE_BULK_READ     9u   //inside readSingleMessageBlocking() - can block ~18 s
#define EXO_STAGE_GET_CONFIG    10u  //inside UART get_config() - can block ~8 s
#define EXO_STAGE_I2C_INIT      11u  //real_time_i2c::init()
#define EXO_STAGE_SETUP_DONE    12u  //end of setup(), about to enter loop()
#define EXO_STAGE_CTOR_EXODATA  13u  //loop() pass 1: constructing ExoData
#define EXO_STAGE_CTOR_COMSMCU  14u  //loop() pass 1: constructing ComsMCU -> runs ExoBLE::setup()
#define EXO_STAGE_BLE_BEGIN     15u  //inside ExoBLE::setup(), around BLE.begin()
#define EXO_STAGE_LINK_STATS    16u  //fetching the Teensy's RT-I2C counters over UART

/**
 * @brief Stamp the current loop() phase into GPREGRET2. Cheap: one register write, no branch.
 */
//Defined in SystemReset.cpp - a .noinit RAM breadcrumb that, unlike GPREGRET2, is expected to
//survive a watchdog reset. See the long comment there.
#if defined(EXO_HAVE_WDT)
void exo_noinit_stage_set(uint8_t stage);
uint8_t exo_noinit_stage_get();
uint8_t exo_noinit_boot_count();
#else
//Teensy has neither a watchdog nor GPREGRET; these compile away to nothing.
inline void exo_noinit_stage_set(uint8_t) {}
inline uint8_t exo_noinit_stage_get() { return 0; }
inline uint8_t exo_noinit_boot_count() { return 0; }
#endif

/**
 * @brief Stamp the current phase. Writes BOTH stores on purpose.
 *
 * GPREGRET2 is kept because it is what carries the link diagnostic through a SREQ (that path is
 * proven to work). The .noinit copy is what should survive a DOG. Reporting both lets the banner
 * show which store actually retained anything, instead of us guessing again.
 */
inline void exo_wdt_stage(uint8_t stage)
{
#if defined(EXO_HAVE_WDT)
    NRF_POWER->GPREGRET2 = (uint32_t)stage;
    exo_noinit_stage_set(stage);
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
    //Latch the RAM copy at the SAME INSTANT as the register copy. This is not optional: B10 reported
    //STAGEn15 because exo_noinit_stage_get() was first reached from exo_reset_reason_string(), deep
    //inside ExoBLE::setup(), by which time THIS boot had already stamped SETUP_ENTRY and then
    //BLE_BEGIN over the previous boot's value - so it reported its own footprint. Both stores are
    //latch-once, so binding them here makes the ordering impossible to get wrong from any caller.
    (void)exo_noinit_stage_get();

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

            //Same hazard, second register user: a BLESTALL marker from our own link-stall detector
            //also lives in GPREGRET and is also read later (from exo_reset_reason_string). Latch it
            //here too or the write below erases the very evidence the detector exists to produce.
            (void)exo_stall_record();

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

//How long the GUI may be silent before we call the link dead. The GUI pings every 2 s, so this is
//four missed pings. Deliberately under the ~9.6 s host-side supervision timeout: when this works we
//are already rebooting before the host even declares the link gone.
#define EXO_BLE_STALL_MS          8000ul

/**
 * @brief Record "the BLE link stalled" and warm-reset. Called from ExoBLE::handle_updates().
 *
 * WHY A WARM RESET: NVIC_SystemReset keeps GPREGRET, so the marker written here survives into the
 * next boot and is reported in the banner. That is the whole point - it turns "we think the link
 * stalls" into a message from the device saying it did.
 */
inline void exo_ble_stall_reset(uint8_t diag)
{
#if defined(EXO_HAVE_WDT)
    const uint8_t marker = (uint8_t)NRF_POWER->GPREGRET;
    const uint8_t count = ((marker & EXO_CRASH_MAGIC_MASK) == EXO_STALL_MAGIC)
                          ? (uint8_t)(marker & EXO_CRASH_COUNT_MASK) : 0u;
    NRF_POWER->GPREGRET = (uint32_t)(EXO_STALL_MAGIC | ((count + 1u) & EXO_CRASH_COUNT_MASK));

    //Park what the link looked like at the moment we gave up, so the next boot can report it. This
    //is the measurement that decides between the two candidate root causes:
    //  - sends BLOCKING (high w) => ArduinoBLE's unbounded `while (_pendingPkt >= _maxPkt) poll();`
    //    (HCI.cpp:636) is engaged: the controller stopped acking and the queue is full.
    //  - sends INSTANT (low w) with a high send count => the stack is writing into a dead link and
    //    never saw the disconnect event. Different bug entirely.
    //GPREGRET2 normally holds the watchdog stage breadcrumb, but the stage is only ever REPORTED on
    //a DOG reset and this is an SREQ, so the two never collide.
    NRF_POWER->GPREGRET2 = (uint32_t)diag;
#else
    (void)diag;
#endif
    exo_system_reset();
}

/**
 * @brief Latched stall marker from the previous boot: count, or 0 if the last reset was not a stall.
 */
inline uint8_t exo_stall_record()
{
#if defined(EXO_HAVE_WDT)
    static bool captured = false;
    static uint8_t count = 0;
    if (!captured)
    {
        captured = true;
        const uint8_t marker = (uint8_t)NRF_POWER->GPREGRET;
        if ((marker & EXO_CRASH_MAGIC_MASK) == EXO_STALL_MAGIC)
        {
            count = (uint8_t)(marker & EXO_CRASH_COUNT_MASK);
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

    exo_wdt_stage(EXO_STAGE_ARMED); //So a trip before the loop's first breadcrumb is identifiable
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
            const uint8_t other_magic = (uint8_t)(latched_marker_raw & EXO_CRASH_MAGIC_MASK);
            if ((other_magic != EXO_WDT_MAGIC) && (other_magic != EXO_STALL_MAGIC))
            {
                NRF_POWER->GPREGRET = 0;
            }
            //DELIBERATELY NOT zeroing GPREGRET2 here. It used to be, and that was the single biggest
            //source of confusion in this investigation: this runs on every boot, from ExoBLE::setup(),
            //and it blanked the watchdog breadcrumb - so any trip between here and the loop's first
            //stamp reported the meaningless STAGE_0. There is no need to clear it: a stale byte is
            //only ever READ as an mbed error code when the CRASH magic is present in GPREGRET, and
            //that branch (above) still clears it.
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
/**
 * @brief ",Bn" - which build is on the board. See EXO_FW_TAG.
 */
//RT-I2C health as reported by the Teensy (which survives a Nano reboot). Defined in
//SystemReset.cpp, filled by UART_command_utils::get_link_stats() during the Nano's setup().
extern float exo_link_stats[6];
extern bool  exo_link_stats_valid;

/**
 * @brief ",I2Cf<frames>_e<errors>_c<code>_t<ms>" - what the TEENSY saw on the RT link.
 *
 * This is the half of the story the Nano cannot tell about itself. c is the last endTransmission()
 * return code (2 = the Nano stopped ACKing on the bus, so it died first; 0 with the Nano dark means
 * frames were still landing in its receive ISR and the MAIN LOOP is what wedged). t is milliseconds
 * since the last SUCCESSFUL transmission.
 */
inline String exo_link_stats_string()
{
    if (!exo_link_stats_valid)
    {
        return String(",I2Cnone");
    }
    //Units match the scaling in UART_command_handlers::get_link_stats: f = thousands of frames,
    //e = tens of errors, t and w = 100 ms units. A value at 300 means "at or above the cap".
    //x = the longest RUN of consecutive failures, in frames. At 100 Hz, 100 of those is one second
    //of total silence - that is an outage. Single digits are the chronic background loss.
    char b[96];
    snprintf(b, sizeof(b), ",I2Cf%luk_e%lux10_c%u_t%lud_w%lud_x%lu",
             (unsigned long)exo_link_stats[0],
             (unsigned long)exo_link_stats[1],
             (unsigned)exo_link_stats[2],
             (unsigned long)exo_link_stats[3],
             (unsigned long)exo_link_stats[4],
             (unsigned long)exo_link_stats[5]);
    return String(b);
}

//The connection parameters the CENTRAL actually chose, captured off the HCI LE Connection
//Complete event by a local patch to the SKETCHBOOK copy of ArduinoBLE (HCI.cpp - see the comment
//block there, and doc section 14.2). extern "C" to avoid depending on C++ mangling across that
//boundary. If ArduinoBLE is ever updated or reinstalled these vanish and the LINK FAILS TO BUILD -
//which is deliberate: a silent revert to "we have no idea what the interval is" is exactly the
//situation section 12 was written about.
extern "C" {
    extern volatile uint16_t exo_ble_cp_interval;
    extern volatile uint16_t exo_ble_cp_latency;
    extern volatile uint16_t exo_ble_cp_timeout;
    extern volatile uint16_t exo_ble_cp_count;
}

/**
 * @brief ",CPi<interval>_l<latency>_t<timeout>_u<0|1>_n<count>" - what the CENTRAL chose.
 *
 * THE POINT OF THIS FIELD: until 2026-09-12 nothing in this firmware had ever read back the
 * connection interval. BLE.setConnectionInterval() sets what is REQUESTED; the central has final
 * say, ArduinoBLE discards its answer, and Windows is documented to accept such a request and then
 * not apply it - randomly. Every "7.5 ms" and "15-30 ms" in the write-ups was a request, not a
 * measurement. This is the measurement.
 *
 * UNITS ARE RAW, as they come off the wire - no conversion, so nothing is lost:
 *   i = interval, 1.25 ms units.  i6 = 7.5 ms (the pinned value), i24 = 30 ms.
 *   l = slave latency, in connection events.
 *   t = supervision timeout, 10 ms units. t500 = 5 s. This is the link's dead-man's switch:
 *       no valid packet from the central within it and the controller must declare the link lost.
 *       Directly relevant to presentation A - see doc section 14.5.
 *   u = would we have sent an L2CAP parameter-update request? Derived from i against our own
 *       compile-time range, using the SAME test as L2CAPSignalingClass::addConnection().
 *       This is the field that separates "a relaxed interval helps" from "not ASKING helps" -
 *       the one alternative an A/B/A cannot rule out on its own.
 *   n = connections since boot. n0 means we have never connected, so i/l/t are meaningless.
 *
 * WHAT IT DOES NOT TELL YOU: whether a later L2CAP update was applied. This is the value the link
 * OPENED at. If the central accepts an update mid-connection, that stays invisible without a
 * sniffer (doc section 14.6).
 */
inline String exo_ble_cp_string()
{
    const uint16_t n = exo_ble_cp_count;
    if (n == 0u)
    {
        return String(",CPnone");
    }

    const uint16_t iv = exo_ble_cp_interval;

    //Mirror addConnection()'s test exactly: a request goes out only when the central's choice falls
    //OUTSIDE our range. Kept in lockstep with the values passed to BLE.setConnectionInterval() in
    //ExoBLE.cpp - if you change one, change the other.
    const uint16_t our_min = EXO_BLE_INTERVAL_PINNED ? 6u : 12u;
    const uint16_t our_max = EXO_BLE_INTERVAL_PINNED ? 6u : 24u;
    const unsigned requested = (iv < our_min || iv > our_max) ? 1u : 0u;

    char b[64];
    snprintf(b, sizeof(b), ",CPi%u_l%u_t%u_u%u_n%u",
             (unsigned)iv,
             (unsigned)exo_ble_cp_latency,
             (unsigned)exo_ble_cp_timeout,
             requested,
             (unsigned)n);
    return String(b);
}

inline String exo_fw_tag_string()
{
    //bN is the .noinit boot counter - see SystemReset.cpp. b1 every time means that RAM is being
    //zeroed at startup and the stage breadcrumb cannot live there.
    //rt1/rt0 = the REAL_TIME_I2C build flag. In the banner on purpose: the bisect build looks
    //broken (no plots, no data) and this is how you tell "the experiment" from "a new fault".
    //rt<n>/fw<n> = the REAL_TIME_I2C and RT_BLE_FORWARD build flags. Both in the banner because
    //the bisect builds look broken in different ways and this is how you tell which is running.
    char tag[40];
    snprintf(tag, sizeof(tag), ",B%u_b%u_rt%u_fw%u",
             (unsigned)EXO_FW_TAG, (unsigned)exo_noinit_boot_count(),
             (unsigned)(REAL_TIME_I2C ? 1u : 0u),
             (unsigned)(RT_BLE_FORWARD ? 1u : 0u));
    return String(tag);
}

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
        return String(head) + names + String(crash) + exo_fw_tag_string();
    }

    //Our own link-stall detector fired on the previous boot. This is the positive confirmation that
    //the BLE link died while the Nano itself was still running - checked before DOG/plain SREQ so it
    //is never mistaken for an End-Trial reboot, which is the other thing that produces SREQ.
    {
        const uint8_t stall_n = exo_stall_record();
        if (stall_n != 0)
        {
            //diag is the byte parked by exo_ble_stall_reset(), latched out of GPREGRET2 by
            //exo_wdt_stage_record() at the top of setup() before anything overwrites it.
            //  w = max writeValue() duration bucket, ~2^(w-1) * 64 us (w=0 means under 64 us)
            //  s = notifications attempted during the silence, bucket ~2^(s-1) (s=0 means none)
            //  _FAIL appended if writeValue() ever returned false
            const uint8_t diag = exo_wdt_stage_record();
            char stall[64];
            if ((diag & 0x80u) == 0u)
            {
                //No marker bit: this byte did not come from exo_ble_link_diag(), so it is a stale
                //stage breadcrumb or a blank. Report that rather than decoding nonsense.
                snprintf(stall, sizeof(stall), ",BLESTALL_n%u_nodiag", (unsigned)stall_n);
            }
            else
            {
                const unsigned w = (unsigned)(diag & 0x0Fu);
                const unsigned sent = (unsigned)((diag >> 5) & 0x03u);
                const bool failed = (diag & 0x10u) != 0u;
                snprintf(stall, sizeof(stall), ",BLESTALL_n%u_w%u_s%u%s",
                         (unsigned)stall_n, w, sent, failed ? "_FAIL" : "");
            }
            return String(head) + names + String(stall) + exo_fw_tag_string();
        }
    }

    //A watchdog reset means the previous boot HUNG rather than faulted, so GPREGRET2 holds a loop()
    //stage breadcrumb instead of an mbed error byte (the crash magic above is what tells them
    //apart). Appended into names the same way CRASH_ is, so the GUI prints it with no change.
    if (reasons & 0x00000002ul)
    {
        //n = the .noinit RAM copy, g = the GPREGRET2 copy. If n is populated while g is 0, that
        //confirms GPREGRET2 is wiped by a watchdog reset and n is the one to trust.
        char stage[48];
        snprintf(stage, sizeof(stage), ",STAGEn%u_g%u_dog%u",
                 (unsigned)exo_noinit_stage_get(),
                 (unsigned)exo_wdt_stage_record(),
                 (unsigned)exo_wdt_boot_count());
        return String(head) + names + String(stage) + exo_fw_tag_string();
    }

    return String(head) + names + exo_fw_tag_string();
}

#endif
