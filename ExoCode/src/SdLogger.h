/**
 * @file SdLogger.h
 *
 * =============================================================================================
 * !!  WARNING - TWO COLUMNS IN THIS LOG ARE MISLEADING. READ BEFORE TRUSTING ANY OUTPUT.  !!
 * =============================================================================================
 *
 *  1. `Current_A` IS A REAL MEASUREMENT, BUT IT IS STALE BY AN UNKNOWN AMOUNT.
 *     It is `motor.i`, decoded in Motor.cpp::read_data() as `int16 * 0.01f` A. That decode is
 *     CORRECT - it matches AK Series Module Product Manual v3.0.0 section 4.3.1 byte for byte.
 *     (An earlier version of this comment claimed the scale was ~6x wrong. That was an artefact
 *     of regressing joint torque on current during CLOSED-LOOP PID operation; retracted. See
 *     "Modification log with claude/Motor-Current-Decode-Investigation.md".)
 *
 *     Two things still make the column look absurd:
 *       a) The status frame is a TIMED BROADCAST on the motor's own clock (manual 4.3.1, and the
 *          "CAN feedback rate" setting), NOT a reply to our command. read_data() pops ONE frame
 *          per motor per cycle from a shared queue routed by QUEUE POSITION, so a row's current
 *          may have been measured many milliseconds before that row's command. Left leg: 60% of
 *          rows carry a fresh frame. Right leg: 15%. A real peak current gets printed next to
 *          eleven unrelated commands.
 *       b) Values above the 10.3 A datasheet peak are genuine - see note 2 for why we routinely
 *          ask for more than that.
 *
 *  2. `Commanded_Torque_Nm` IS NEITHER Nm NOR AMPS.
 *     It is `motor.last_command` = `i_sat`, a number in fictitious "firmware amps" in the MOTOR
 *     frame. The firmware packs it against +-_I_MAX (10.3, the datasheet PEAK CURRENT), but the
 *     motor unpacks that field against +-12.0 (manual 4.2 parameter table, AK60-6). So the motor
 *     receives  i_sat * 12.0/10.3 = i_sat * 1.165  in ITS units, which the manual documents as
 *     N.m at the motor output shaft.
 *         true motor-shaft torque = Commanded_Torque_Nm * 1.165
 *         true joint torque       = Commanded_Torque_Nm * 1.165 * 4.5 = * 5.243
 *     It is motor-frame, so on the flipped side (ankleFlipMotorDir = right) its sign is opposite
 *     to `Current_A`, which is joint-frame. Comparing the two directly on the right leg gives a
 *     spurious anti-correlation.
 *
 *  NONE OF THIS MATTERS IN PRACTICE RIGHT NOW: SD logging is disabled (it cost too much control
 *  loop time - see Spline-Jitter-Round-2-SD-Logging-Regression.md), so these columns are not being
 *  produced. This block exists so that whoever turns the logger back on, or reads an archived log,
 *  is not misled. Fix the columns before trusting them; do not fix them on the strength of an old
 *  log alone.
 *
 *  The GUI CSV has superseded this for the question that actually matters. Channels 8/9
 *  ("Commanded Torque L/R") stream `motor.t_ff * gearing` -- genuine joint N.m as the firmware
 *  reckons it, NOT i_sat, so they do NOT carry the amps/Nm confusion of item 2 above. They sit
 *  next to channels 1/3 ("Measured Torque"), which are the ankle torque sensor. Commanded vs
 *  measured, both fresh, both ~100 Hz, no staleness. That pair measures the real delivered-torque
 *  ratio directly.
 *
 *  Caveat on 8/9: t_ff is what the firmware INTENDS to send. What the motor actually receives is
 *  i_sat scaled by 12.0/10.3, so the true command is ~4.9% above channel 8 (if the MIT field is
 *  N.m). Small, but it means 8/9 are not an independent check on the +-10.3 vs +-12.0 question --
 *  only the blocked-joint test in Motor-Current-Decode-Investigation.md sec 5.2 settles that.
 * =============================================================================================
 *
 * @brief Onboard SD-card data logger for the OpenExo Teensy.
 *
 * Writes per-motor and per-ground-strike .txt logs during a trial, matching the hip-exo
 * log format. A fresh session folder is opened automatically on the trial-on edge,
 * decimated motor rows (~100 Hz) and per-ground-strike step rows are written, and
 * files are flushed on a staggered cadence so no single 500 Hz control cycle stalls.
 *
 * Design: docs/superpowers/specs/2026-07-06-teensy-sd-logging-design.md
 */

#ifndef SDLOGGER_H
#define SDLOGGER_H

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include <Arduino.h>
#include <SD.h>
#include "SdRingBuffer.h"
#include "ExoData.h"
#include "SideData.h"
#include "JointData.h"
#include "StatusDefs.h"

// ---- Compile-time config ----
#define ENABLE_SD_LOGGING     1      // 0 compiles the logger out entirely
#define SD_LOG_BASE_PATH      "/EXOLOG"
#define SD_LOG_DEBUG          1      // 1 -> ran/s + maxLoop/maxSD timing to Serial (1s) + /EXOLOG/debug_log.txt (10s). For diagnosing loop stalls.
#define SD_LOG_SELFTEST       0      // 1 -> at boot, run a SAFE raw-SD write test (no motors, no trial) then HALT.
                                     //      Writes results to /EXOLOG/SELFTEST/. Set back to 0 for normal operation.
#define SD_LOG_SELFTEST_TRIAL 0      // 1 -> at boot, drive the REAL logger update() path with a FAKE trial (no
                                     //      motors, never enabled) then HALT. Writes a normal /EXOLOG/000N/ folder.
                                     //      Set back to 0 for normal operation.

// ---- Runtime defaults (overridable via the [Logging] section of /config.ini; no reflash) ----
// config.ini keys: sdLogEnabled, sdLogDecimation, sdLogFlushMs
#define SD_LOG_DEFAULT_ENABLED     1     // log to SD during trials
#define SD_LOG_DEFAULT_DECIMATION  5     // every Nth 500Hz cycle (5 -> ~100Hz)
#define SD_LOG_DEFAULT_FLUSH_MS    300   // per-tick flush cadence (ms)

class SdLogger
{
    public:
        SdLogger(ExoData* data);

        /**
         * @brief Drive the logger. Call once per .ino loop iteration.
         * @param ran The Exo::run() return value (true exactly once per control cycle).
         * Never blocks control; no-ops unless a trial is active and the SD is usable.
         */
        void update(bool ran);

        // Safe boot-time SD write test (no motors, no trial). Called from setup() when
        // SD_LOG_SELFTEST==1. Writes /EXOLOG/SELFTEST/ and reads it back for verification.
        static void self_test();

        // Flush + close any open session immediately, from anywhere (e.g. the reset handler,
        // which runs inside exo.run() and would otherwise reboot before update() gets to
        // close the files -> abandoned/corrupted logs). Safe to call even if no session is open.
        static void close_active();

    private:
        static SdLogger* _instance;   // the (single) live logger, for close_active()
        ExoData* _data;
        bool     _available;      // SD mounted & usable
        bool     _logging;        // a session file set is open
        bool     _prev_active;    // for trial-edge detection
        uint16_t _session_index;  // next session folder index
        uint8_t  _decim;          // decimation counter
        uint32_t _last_flush_ms;
        uint8_t  _flush_turn;     // rotates 0..2 across the three files

        // Runtime tunables, loaded from [Logging] in /config.ini at boot (see defaults above)
        bool     _enabled;
        uint8_t  _decimation;
        uint32_t _flush_tick_ms;

        // Diagnostics (printed once/sec when SD_LOG_DEBUG=1)
        uint32_t _dbg_ran;      // ran==true update calls during logging, per interval
        uint32_t _dbg_bytes;    // motor-row bytes written, per interval
        uint32_t _dbg_last_ms;
        uint32_t _dbg_max_loop_us;  // worst full loop-iteration time, per debug interval
        uint32_t _dbg_max_sd_us;    // worst time spent in the SD ops (drain+sync), per debug interval
        uint32_t _dbg_last_loop_us; // timestamp of the previous update() call, for the loop-time delta

        FsFile   _file[4];             // 0=Motor_L, 1=Motor_R, 2=Ground_strike, 3=debug_log
        SdRingBuffer _rb[4];           // one ring buffer per file (DMAMEM-backed storage)
        uint64_t _bytes_written[4];    // logical bytes written per file, for truncate() at close
        uint8_t  _drain_turn;          // round-robins the single-sector drain across the files

        void       _begin_if_needed();
        void       _load_config();
        uint16_t   _scan_next_index();
        void       _handle_session();
        void       _open_session();
        void       _close_session();
        JointData* _used_joint(SideData& side);
        void       _write_motor_row(int idx, SideData& side, const char* label);  // push a row into ring buffer idx
        void       _check_ground_strike_events();
        void       _service_writes();                    // isBusy()-gated single-sector drain
        uint64_t   _prealloc_bytes(bool motor) const;    // adaptive contiguous pre-allocation size
        void       _maybe_flush();                       // durability sync, gated on !isBusy()
};

#endif
#endif
