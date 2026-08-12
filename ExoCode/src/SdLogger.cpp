#include "SdLogger.h"
#include "IniFile.h"
#include "ParseIni.h"   // for ini_config::buffer_length (line length when scanning config.ini)

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

SdLogger* SdLogger::_instance = nullptr;

// Per-file ring-buffer storage in OCRAM (DMAMEM). Sized to ride out a multi-hundred-ms card stall
// at the log data rate (see design doc). ~18 KB total.
DMAMEM static uint8_t s_sdlog_buf_l[8192];
DMAMEM static uint8_t s_sdlog_buf_r[8192];
DMAMEM static uint8_t s_sdlog_buf_gs[2048];
DMAMEM static uint8_t s_sdlog_buf_dbg[2048];   // control-loop debug stream (index 3)

SdLogger::SdLogger(ExoData* data)
: _data(data), _available(false), _logging(false), _prev_active(false),
  _session_index(1), _decim(0), _last_flush_ms(0), _flush_turn(0),
  _enabled(SD_LOG_DEFAULT_ENABLED != 0),
  _decimation(SD_LOG_DEFAULT_DECIMATION),
  _flush_tick_ms(SD_LOG_DEFAULT_FLUSH_MS),
  _dbg_ran(0), _dbg_bytes(0), _dbg_last_ms(0)
{
    _instance = this;
    _drain_turn = 0;
    _dbg_max_loop_us = 0;
    _dbg_max_sd_us = 0;
    _dbg_last_loop_us = 0;
    _bytes_written[0] = _bytes_written[1] = _bytes_written[2] = _bytes_written[3] = 0;
}

void SdLogger::close_active()
{
    // Flush + close any open session right now, independent of the trial_off/status path.
    // Called from the system-reset handler so a reboot never leaves the three log files
    // open (which corrupts the FAT / cross-links clusters). No-op if nothing is logging.
    if (_instance != nullptr && _instance->_logging)
    {
        _instance->_close_session();
    }
}

void SdLogger::update(bool ran)
{
#if ENABLE_SD_LOGGING
    _begin_if_needed();
    if (!_available || !_enabled) return;

    _handle_session();

#if SD_LOG_DEBUG
    // Worst-case full loop-iteration time = interval between update() calls (= one .ino loop pass:
    // exo.run() + this update()). This is the stall detector.
    {
        const uint32_t now_us = micros();
        const uint32_t loop_us = now_us - _dbg_last_loop_us;
        _dbg_last_loop_us = now_us;
        if (loop_us < 1000000UL && loop_us > _dbg_max_loop_us) _dbg_max_loop_us = loop_us;
    }
    if (ran) _dbg_ran++;
    if (millis() - _dbg_last_ms > 1000)
    {
        _dbg_last_ms = millis();
        char line[200];
        snprintf(line, sizeof(line),
            "t=%lums status=%u logging=%d ran/s=%lu bytes/s=%lu maxLoop=%luus maxSD=%luus L.used=%d\n",
            (unsigned long)millis(), (unsigned)_data->get_status(), (int)_logging,
            (unsigned long)_dbg_ran, (unsigned long)_dbg_bytes,
            (unsigned long)_dbg_max_loop_us, (unsigned long)_dbg_max_sd_us,
            (int)_data->left_side.ankle.is_used);
        Serial.print("SdLog: "); Serial.print(line);
        // Route the debug line into its own non-blocking stream (index 3 -> debug_log.txt in the
        // session folder, separate from the motor logs so downstream parsers are untouched). push()
        // is a safe no-op when no session is open, so idle-boot debug is serial-only. No blocking
        // SD.open on the control path.
        _rb[3].push((const uint8_t*)line, strlen(line));
        // Read the two maxes together: maxLoop ~= maxSD => the SD ops ARE the stall; maxLoop >> maxSD
        // => the stall is in exo.run() (control/CAN/UART), not the logger.
        _dbg_ran = 0; _dbg_bytes = 0; _dbg_max_loop_us = 0; _dbg_max_sd_us = 0;
        // Exclude this once/1s print (+ once/10s blocking file write) from the next loop-time sample,
        // so maxLoop reflects only real control-loop stalls, not our own diagnostic overhead.
        _dbg_last_loop_us = micros();
    }
#endif

    // Producer: format rows into the RAM ring buffers only on a real control cycle. Never touches SD.
    if (_logging && ran)
    {
        _check_ground_strike_events();   // every control cycle so strikes aren't missed
        if (++_decim >= _decimation)
        {
            _decim = 0;
            _write_motor_row(0, _data->left_side,  "L");
            _write_motor_row(1, _data->right_side, "R");
        }
    }

    // Drainer + durability sync: run every loop iteration, gated on the card being free, so the ring
    // buffers empty whenever the card is idle and no control cycle ever waits on SD.
#if SD_LOG_DEBUG
    const uint32_t sd_t0 = micros();
#endif
    _service_writes();
    _maybe_flush();
#if SD_LOG_DEBUG
    const uint32_t sd_dt = micros() - sd_t0;
    if (sd_dt > _dbg_max_sd_us) _dbg_max_sd_us = sd_dt;
#endif
#else
    (void)ran;
#endif
}

void SdLogger::_begin_if_needed()
{
    static bool tried = false;
    if (tried) return;
    tried = true;

    _available = SD.begin(BUILTIN_SDCARD);   // idempotent; card already used for config
    if (_available)
    {
        _load_config();                      // override defaults from [Logging] in /config.ini
        _session_index = _scan_next_index();
#if SD_LOG_DEBUG
        SD.remove(SD_LOG_BASE_PATH "/debug_log.txt");   // start each boot with a fresh debug log
#endif
    }

#if SD_LOG_DEBUG
    Serial.print("SdLogger: SD ");
    if (_available)
    {
        Serial.print("ok | enabled="); Serial.print(_enabled);
        Serial.print(" decim=");       Serial.print(_decimation);
        Serial.print(" flushMs=");     Serial.print(_flush_tick_ms);
        Serial.print(" next index ");  Serial.println(_session_index);
    }
    else { Serial.println("FAIL"); }
#endif
}

void SdLogger::_load_config()
{
    // Read overrides from the [Logging] section of /config.ini. Missing file/section/keys
    // leave the compile-time defaults in place, so older SD cards keep working.
    IniFile ini("/config.ini");
    if (!ini.open()) return;

    // Buffer must hold the longest line in config.ini while scanning (comments are long),
    // so match the main parser's size rather than the short key/value length.
    char buf[ini_config::buffer_length];
    int v;
    if (ini.getValue("Logging", "sdLogEnabled",    buf, sizeof(buf), v)) _enabled    = (v != 0);
    if (ini.getValue("Logging", "sdLogDecimation", buf, sizeof(buf), v)) _decimation = (v >= 1) ? (uint8_t)v : 1;
    if (ini.getValue("Logging", "sdLogFlushMs",    buf, sizeof(buf), v)) _flush_tick_ms = (v >= 1) ? (uint32_t)v : 1;

    ini.close();
}

uint16_t SdLogger::_scan_next_index()
{
    char path[32];
    for (uint16_t i = 1; i < 9999; ++i)
    {
        snprintf(path, sizeof(path), "%s/%04u", SD_LOG_BASE_PATH, i);
        if (!SD.exists(path)) return i;
    }
    return 9999;
}

void SdLogger::_handle_session()
{
    // Log whenever the controller is actually running, matching the firmware's own
    // "active_trial" definition (Motor.cpp / Controller.cpp): trial_on OR the FSR
    // calibration/refinement states (motors are commanded in all of these). Keying on
    // trial_on alone missed real sessions, since the exo runs the controller during
    // fsr_refinement and may never formally reach trial_on.
    const uint16_t s = _data->get_status();
    const bool active =
        ((s == status_defs::messages::trial_on) ||
         (s == status_defs::messages::fsr_calibration) ||
         (s == status_defs::messages::fsr_refinement)) && !_data->estop;

    if (active && !_prev_active)      _open_session();
    else if (!active && _prev_active) _close_session();
    _prev_active = active;
}

void SdLogger::_open_session()
{
    char dir[32];
    SD.mkdir(SD_LOG_BASE_PATH);
    snprintf(dir, sizeof(dir), "%s/%04u", SD_LOG_BASE_PATH, _session_index);
    SD.mkdir(dir);

    const char* names[4] = { "Motor_L_log.txt", "Motor_R_log.txt", "Ground_strike_log.txt", "debug_log.txt" };
    uint8_t* stores[4]   = { s_sdlog_buf_l, s_sdlog_buf_r, s_sdlog_buf_gs, s_sdlog_buf_dbg };
    size_t   caps[4]     = { sizeof(s_sdlog_buf_l), sizeof(s_sdlog_buf_r), sizeof(s_sdlog_buf_gs), sizeof(s_sdlog_buf_dbg) };

    char path[64];
    for (int i = 0; i < 4; ++i)
    {
        snprintf(path, sizeof(path), "%s/%s", dir, names[i]);
        _file[i] = SD.sdfs.open(path, O_WRONLY | O_CREAT | O_TRUNC);
        if (!_file[i]) { _close_session(); return; }
        _file[i].preAllocate(_prealloc_bytes(i < 2));   // false return is fine: falls back to on-demand growth
        _rb[i].init(stores[i], caps[i]);
        _bytes_written[i] = 0;
    }

    // Queue headers into the ring buffers (drained as normal data). Same format/columns as before.
    // NOTE: two of these column names lie - see the WARNING block at the top of SdLogger.h.
    //   Commanded_Torque_Nm is motor.last_command = i_sat, i.e. AMPS in the MOTOR frame
    //                       (x 4.995 for joint Nm; sign is flipped vs Current_A on the right leg)
    //   Current_A           is motor.i, whose decode is WRONG by ~6x. DO NOT USE.
    // Names are kept as-is so existing parsers of old logs still work.
    const char* motor_hdr =
        "Motor,Teensy_time_s,Status,Gait_phase,Position_rad,Velocity_rad_s,Torque_Nm,"
        "Commanded_Torque_Nm,Current_A,Filtered_Torque_Nm,Desired_Torque_Nm,"
        "Toe_FSR,Stance,Enabled,Timeout_ct,Error\n";
    char hbuf[128];
    for (int i = 0; i < 2; ++i)
    {
        int n = snprintf(hbuf, sizeof(hbuf), "# OpenExo SD log rate~%dHz t0_us=%lu\n",
                         500 / _decimation, (unsigned long)micros());
        if (n > 0) _rb[i].push((const uint8_t*)hbuf, (size_t)n);
        _rb[i].push((const uint8_t*)motor_hdr, strlen(motor_hdr));
    }
    const char* gs_hdr =
        "# OpenExo ground-strike log (toe-FSR strike onset; heel unused)\n"
        "Leg,Teensy_time_s,Prev_step_ms,Expected_step_ms\n";
    _rb[2].push((const uint8_t*)gs_hdr, strlen(gs_hdr));

    const char* dbg_hdr = "# OpenExo control-loop debug (SD_LOG_DEBUG): ran/s, maxLoop us, maxSD us\n";
    _rb[3].push((const uint8_t*)dbg_hdr, strlen(dbg_hdr));

    _decim = 0;
    _flush_turn = 0;
    _drain_turn = 0;
    _last_flush_ms = millis();
    _logging = true;
    _session_index++;   // next trial -> fresh folder

#if SD_LOG_DEBUG
    Serial.print("SdLogger: opened "); Serial.println(dir);
#endif
}

void SdLogger::_close_session()
{
    for (int i = 0; i < 4; ++i)
    {
        if (!_file[i]) continue;
        // Flush whatever remains in the ring buffer (blocking is fine here: the trial is ending).
        const uint8_t* p; size_t n;
        while ((n = _rb[i].peek(&p)) > 0)
        {
            size_t w = _file[i].write(p, n);
            _bytes_written[i] += w;
            _rb[i].consume(w);
            if (w < n) break;   // write error; stop rather than spin
        }
        _file[i].truncate(_bytes_written[i]);   // discard the unused pre-allocated tail
        _file[i].sync();
        _file[i].close();
    }
    _logging = false;
#if SD_LOG_DEBUG
    Serial.println("SdLogger: closed session");
#endif
}

JointData* SdLogger::_used_joint(SideData& side)
{
    if (side.ankle.is_used) return &side.ankle;
    if (side.hip.is_used)   return &side.hip;
    if (side.knee.is_used)  return &side.knee;
    if (side.elbow.is_used) return &side.elbow;
    return nullptr;
}

void SdLogger::_write_motor_row(int idx, SideData& side, const char* label)
{
    JointData* j = _used_joint(side);
    if (j == nullptr) return;

    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "%s,%.4f,%u,%.2f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
        label,
        micros() / 1.0e6,
        (unsigned)_data->get_status(),
        side.percent_gait,
        j->motor.p, j->motor.v,
        j->torque_reading, j->motor.last_command, j->motor.i,
        j->controller.filtered_torque_reading, j->controller.desired_torque,
        side.toe_fsr, (int)side.toe_stance,
        (int)j->motor.enabled, j->motor.timeout_count,
        _data->error_code);

    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    _rb[idx].push((const uint8_t*)buf, (size_t)n);
}

void SdLogger::_check_ground_strike_events()
{
    if (!_logging) return;
    const float t = micros() / 1.0e6;
    char buf[64];

    if (_data->left_side.ground_strike)
    {
        int n = snprintf(buf, sizeof(buf), "L,%.4f,%.1f,%.1f\n", t,
                         _data->left_side.last_step_duration, _data->left_side.expected_step_duration);
        if (n > 0) _rb[2].push((const uint8_t*)buf, (size_t)n);
    }
    if (_data->right_side.ground_strike)
    {
        int n = snprintf(buf, sizeof(buf), "R,%.4f,%.1f,%.1f\n", t,
                         _data->right_side.last_step_duration, _data->right_side.expected_step_duration);
        if (n > 0) _rb[2].push((const uint8_t*)buf, (size_t)n);
    }
}

uint64_t SdLogger::_prealloc_bytes(bool motor) const
{
    if (!motor) return (uint64_t)1 << 20;   // ground-strike: fixed 1 MB (event-driven, low volume)
    const uint32_t ROW_BYTES_EST = 72;
    uint32_t rows_per_s = 500u / (_decimation ? _decimation : 1);
    uint64_t want = (uint64_t)rows_per_s * ROW_BYTES_EST * 5400ull;   // ~90 min at the actual rate
    const uint64_t MINB = (uint64_t)1 << 20;
    const uint64_t MAXB = (uint64_t)128 << 20;
    if (want < MINB) want = MINB;
    if (want > MAXB) want = MAXB;
    return want;
}

void SdLogger::_service_writes()
{
    if (!_logging || !_available) return;
    if (SD.sdfs.card()->isBusy()) return;    // never wait on the card

    for (int attempt = 0; attempt < 4; ++attempt)
    {
        _drain_turn = (uint8_t)((_drain_turn + 1) % 4);
        int i = _drain_turn;
        if (!_file[i]) continue;

        // If bytes were dropped on overflow, record a visible gap marker (direct write, still gated).
        if (_rb[i].dropped() > 0)
        {
            char g[48];
            int gn = snprintf(g, sizeof(g), "\n# GAP %lu bytes\n", (unsigned long)_rb[i].dropped());
            if (gn > 0) { _bytes_written[i] += _file[i].write((const uint8_t*)g, (size_t)gn); }
            _rb[i].clear_dropped();
        }

        if (_rb[i].size() >= 512)
        {
            const uint8_t* p;
            size_t avail = _rb[i].peek(&p);            // largest contiguous run at the tail
            size_t w = (avail >= 512) ? 512 : avail;   // one sector; wrap remainder drains next turn
            size_t wrote = _file[i].write(p, w);
            _rb[i].consume(wrote);
            _bytes_written[i] += wrote;
            _dbg_bytes += (uint32_t)wrote;
            return;                                    // at most one sector write per call
        }
    }
}

void SdLogger::_maybe_flush()
{
    if (!_logging) return;
    const uint32_t now = millis();
    if (now - _last_flush_ms < _flush_tick_ms) return;
    if (SD.sdfs.card()->isBusy()) return;    // defer; don't stall control on a sync
    _last_flush_ms = now;

    if (_file[_flush_turn]) _file[_flush_turn].sync();   // one file's directory sync per tick
    _flush_turn = (uint8_t)((_flush_turn + 1) % 4);
}

// ---------------------------------------------------------------------------
// Boot-time SD write self-test. No motors, no trial. Writes a single-file case
// (control) and a three-concurrent-file case (mirrors the real logger), then
// reads back the line counts. Results go to Serial AND /EXOLOG/SELFTEST/result.txt
// so the card can be checked offline without a serial connection.
// ---------------------------------------------------------------------------
void SdLogger::self_test()
{
    const int N = 300;   // rows written per file

    delay(800);
    Serial.println();
    Serial.println("=== SdLogger self-test (no motors, no trial) ===");

    if (!SD.begin(BUILTIN_SDCARD)) { Serial.println("SD.begin FAILED"); return; }
    SD.mkdir(SD_LOG_BASE_PATH);
    SD.mkdir(SD_LOG_BASE_PATH "/SELFTEST");

    // Clear prior results so line counts are clean (FILE_WRITE appends).
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/single.txt");
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/multi_A.txt");
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/multi_B.txt");
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/multi_C.txt");
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/conc_A.txt");
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/conc_B.txt");
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/conc_C.txt");
    SD.remove(SD_LOG_BASE_PATH "/SELFTEST/result.txt");

    // --- Case A: one file open at a time (control) ---
    {
        File f = SD.open(SD_LOG_BASE_PATH "/SELFTEST/single.txt", FILE_WRITE);
        if (f)
        {
            f.println("idx,a,b");
            for (int i = 0; i < N; i++)
            {
                char buf[48];
                int n = snprintf(buf, sizeof(buf), "%d,%.3f,%.3f\n", i, i * 0.01, i * 0.02);
                f.write((const uint8_t*)buf, (size_t)n);
                if ((i % 50) == 0) f.flush();
            }
            f.flush();
            f.close();
        }
        else { Serial.println("A: open single.txt FAILED"); }
    }

    // --- Case B: three files open at once, interleaved writes + rotating flush ---
    //     (exactly the pattern the real logger uses)
    {
        File a = SD.open(SD_LOG_BASE_PATH "/SELFTEST/multi_A.txt", FILE_WRITE);
        File b = SD.open(SD_LOG_BASE_PATH "/SELFTEST/multi_B.txt", FILE_WRITE);
        File c = SD.open(SD_LOG_BASE_PATH "/SELFTEST/multi_C.txt", FILE_WRITE);
        if (a && b && c)
        {
            a.println("idx,a,b"); b.println("idx,a,b"); c.println("idx,a,b");
            for (int i = 0; i < N; i++)
            {
                char buf[48];
                int n = snprintf(buf, sizeof(buf), "%d,%.3f,%.3f\n", i, i * 0.01, i * 0.02);
                a.write((const uint8_t*)buf, (size_t)n);
                b.write((const uint8_t*)buf, (size_t)n);
                c.write((const uint8_t*)buf, (size_t)n);
                switch (i % 3) { case 0: a.flush(); break; case 1: b.flush(); break; case 2: c.flush(); break; }
            }
            a.flush(); b.flush(); c.flush();
            a.close(); b.close(); c.close();
        }
        else { Serial.println("B: open multi_*.txt FAILED"); }
    }

    // --- Case C: three files open, but with SD.begin() + a config read fired every
    //     50 rows, mimicking set_controller_params()/ParamsFromSD touching the SD
    //     mid-trial. This is the suspected real-world failure mode. ---
    {
        File a = SD.open(SD_LOG_BASE_PATH "/SELFTEST/conc_A.txt", FILE_WRITE);
        File b = SD.open(SD_LOG_BASE_PATH "/SELFTEST/conc_B.txt", FILE_WRITE);
        File c = SD.open(SD_LOG_BASE_PATH "/SELFTEST/conc_C.txt", FILE_WRITE);
        if (a && b && c)
        {
            a.println("idx,a,b"); b.println("idx,a,b"); c.println("idx,a,b");
            for (int i = 0; i < N; i++)
            {
                char buf[48];
                int n = snprintf(buf, sizeof(buf), "%d,%.3f,%.3f\n", i, i * 0.01, i * 0.02);
                a.write((const uint8_t*)buf, (size_t)n);
                b.write((const uint8_t*)buf, (size_t)n);
                c.write((const uint8_t*)buf, (size_t)n);
                switch (i % 3) { case 0: a.flush(); break; case 1: b.flush(); break; case 2: c.flush(); break; }

                if (i > 0 && (i % 50) == 0)   // simulate a controller-param load hitting the SD
                {
                    SD.begin(BUILTIN_SDCARD);
                    File cfg = SD.open("/config.ini", FILE_READ);
                    if (cfg) { while (cfg.available()) cfg.read(); cfg.close(); }
                }
            }
            a.flush(); b.flush(); c.flush();
            a.close(); b.close(); c.close();
        }
        else { Serial.println("C: open conc_*.txt FAILED"); }
    }

    // --- Read back line counts (one file open at a time to avoid confounding) ---
    const char* names[7] = {
        SD_LOG_BASE_PATH "/SELFTEST/single.txt",
        SD_LOG_BASE_PATH "/SELFTEST/multi_A.txt",
        SD_LOG_BASE_PATH "/SELFTEST/multi_B.txt",
        SD_LOG_BASE_PATH "/SELFTEST/multi_C.txt",
        SD_LOG_BASE_PATH "/SELFTEST/conc_A.txt",
        SD_LOG_BASE_PATH "/SELFTEST/conc_B.txt",
        SD_LOG_BASE_PATH "/SELFTEST/conc_C.txt",
    };
    long counts[7] = {0, 0, 0, 0, 0, 0, 0};
    for (int k = 0; k < 7; k++)
    {
        File f = SD.open(names[k], FILE_READ);
        if (f) { while (f.available()) { if (f.read() == '\n') counts[k]++; } f.close(); }
        Serial.print(names[k]); Serial.print(" -> "); Serial.print(counts[k]); Serial.println(" lines");
    }

    File res = SD.open(SD_LOG_BASE_PATH "/SELFTEST/result.txt", FILE_WRITE);
    if (res)
    {
        res.print("expected ~"); res.print(N + 1); res.println(" lines per file (1 header + rows)");
        res.println("single/multi = isolated writes; conc = writes with SD.begin()+config reads interleaved");
        for (int k = 0; k < 7; k++) { res.print(names[k]); res.print(" -> "); res.print(counts[k]); res.println(" lines"); }
        res.flush();
        res.close();
    }

    Serial.print("Expected ~"); Serial.print(N + 1); Serial.println(" lines each.");
    Serial.println("single vs multi tells us if concurrent write handles are the problem.");
    Serial.println("=== self-test done. Power off and read /EXOLOG/SELFTEST/result.txt ===");
}

#endif
