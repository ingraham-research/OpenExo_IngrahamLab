/**
 * @file SdLogger.h
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
#include "ExoData.h"
#include "SideData.h"
#include "JointData.h"
#include "StatusDefs.h"

// ---- Compile-time config ----
#define ENABLE_SD_LOGGING     1      // 0 compiles the logger out entirely
#define SD_LOG_BASE_PATH      "/EXOLOG"
#define SD_LOG_DEBUG          1      // 1 -> once/sec, log ran/bytes/used to Serial AND /EXOLOG/debug_log.txt
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

    private:
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

        File _f_motor_l;
        File _f_motor_r;
        File _f_gs;

        void       _begin_if_needed();
        void       _load_config();
        uint16_t   _scan_next_index();
        void       _handle_session();
        void       _open_session();
        void       _close_session();
        JointData* _used_joint(SideData& side);
        int        _write_motor_row(File& f, SideData& side, const char* label);  // returns bytes written
        void       _check_ground_strike_events();
        void       _maybe_flush();
};

#endif
#endif
