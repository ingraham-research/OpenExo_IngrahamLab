#include "SdLogger.h"
#include "IniFile.h"
#include "ParseIni.h"   // for ini_config::buffer_length (line length when scanning config.ini)

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

SdLogger* SdLogger::_instance = nullptr;

SdLogger::SdLogger(ExoData* data)
: _data(data), _available(false), _logging(false), _prev_active(false),
  _session_index(1), _decim(0), _last_flush_ms(0), _flush_turn(0),
  _enabled(SD_LOG_DEFAULT_ENABLED != 0),
  _decimation(SD_LOG_DEFAULT_DECIMATION),
  _flush_tick_ms(SD_LOG_DEFAULT_FLUSH_MS),
  _dbg_ran(0), _dbg_bytes(0), _dbg_last_ms(0)
{
    _instance = this;
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
    // Count and print EVERY second for the whole run (not just while logging), so we can
    // see the true loop rate and whether the trial state (status) is stable over time.
    // status: 2 == trial_on (see status_defs::messages). Window is a full ~1s except the
    // very first line, so ran/s is a real rate.
    if (ran) _dbg_ran++;
    if (millis() - _dbg_last_ms > 1000)
    {
        _dbg_last_ms = millis();
        char line[176];
        snprintf(line, sizeof(line),
            "t=%lums status=%u logging=%d ran/s=%lu bytes/s=%lu L.used=%d fL=%d\n",
            (unsigned long)millis(), (unsigned)_data->get_status(), (int)_logging,
            (unsigned long)_dbg_ran, (unsigned long)_dbg_bytes,
            (int)_data->left_side.ankle.is_used, (int)(bool)_f_motor_l);
        Serial.print("SdLog: "); Serial.print(line);
        // Persist to the card (open/append/close) so it's readable offline after a trial.
        File dbg = SD.open(SD_LOG_BASE_PATH "/debug_log.txt", FILE_WRITE);
        if (dbg) { dbg.print(line); dbg.flush(); dbg.close(); }
        _dbg_ran = 0; _dbg_bytes = 0;
    }
#endif

    if (!_logging || !ran) return;

    _check_ground_strike_events();   // every control cycle so strikes aren't missed

    if (++_decim >= _decimation)
    {
        _decim = 0;
        _dbg_bytes += (uint32_t)_write_motor_row(_f_motor_l, _data->left_side,  "L");
        _dbg_bytes += (uint32_t)_write_motor_row(_f_motor_r, _data->right_side, "R");
    }

    _maybe_flush();
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

    char path[48];
    snprintf(path, sizeof(path), "%s/Motor_L_log.txt", dir);
    _f_motor_l = SD.open(path, FILE_WRITE);
    snprintf(path, sizeof(path), "%s/Motor_R_log.txt", dir);
    _f_motor_r = SD.open(path, FILE_WRITE);
    snprintf(path, sizeof(path), "%s/Ground_strike_log.txt", dir);
    _f_gs = SD.open(path, FILE_WRITE);

    if (!_f_motor_l || !_f_motor_r || !_f_gs) { _close_session(); return; }

    const char* motor_hdr =
        "Motor,Teensy_time_s,Status,Gait_phase,Position_rad,Velocity_rad_s,Torque_Nm,"
        "Commanded_Torque_Nm,Current_A,Filtered_Torque_Nm,Desired_Torque_Nm,"
        "Toe_FSR,Stance,Enabled,Timeout_ct,Error";

    _f_motor_l.print("# OpenExo SD log rate~"); _f_motor_l.print(500 / _decimation);
    _f_motor_l.print("Hz t0_us=");              _f_motor_l.println(micros());
    _f_motor_l.println(motor_hdr);

    _f_motor_r.print("# OpenExo SD log rate~"); _f_motor_r.print(500 / _decimation);
    _f_motor_r.print("Hz t0_us=");              _f_motor_r.println(micros());
    _f_motor_r.println(motor_hdr);

    _f_gs.println("# OpenExo ground-strike log (toe-FSR strike onset; heel unused)");
    _f_gs.println("Leg,Teensy_time_s,Prev_step_ms,Expected_step_ms");

    _decim = 0;
    _flush_turn = 0;
    _last_flush_ms = millis();
    _logging = true;
    _session_index++;   // next trial -> fresh folder

#if SD_LOG_DEBUG
    Serial.print("SdLogger: opened "); Serial.println(dir);
#endif
}

void SdLogger::_close_session()
{
    if (_f_motor_l) { _f_motor_l.flush(); _f_motor_l.close(); }
    if (_f_motor_r) { _f_motor_r.flush(); _f_motor_r.close(); }
    if (_f_gs)      { _f_gs.flush();      _f_gs.close(); }
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

int SdLogger::_write_motor_row(File& f, SideData& side, const char* label)
{
    JointData* j = _used_joint(side);
    if (j == nullptr || !f) return 0;

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

    if (n <= 0) return 0;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    return (int)f.write((const uint8_t*)buf, (size_t)n);
}

void SdLogger::_check_ground_strike_events()
{
    if (!_f_gs) return;
    const float t = micros() / 1.0e6;

    if (_data->left_side.ground_strike)
    {
        _f_gs.print("L,");  _f_gs.print(t, 4); _f_gs.print(",");
        _f_gs.print(_data->left_side.last_step_duration, 1); _f_gs.print(",");
        _f_gs.println(_data->left_side.expected_step_duration, 1);
    }
    if (_data->right_side.ground_strike)
    {
        _f_gs.print("R,");  _f_gs.print(t, 4); _f_gs.print(",");
        _f_gs.print(_data->right_side.last_step_duration, 1); _f_gs.print(",");
        _f_gs.println(_data->right_side.expected_step_duration, 1);
    }
}

void SdLogger::_maybe_flush()
{
    const uint32_t now = millis();
    if (now - _last_flush_ms < _flush_tick_ms) return;
    _last_flush_ms = now;

    switch (_flush_turn)   // one sync per tick, rotating across files
    {
        case 0: if (_f_motor_l) _f_motor_l.flush(); break;
        case 1: if (_f_motor_r) _f_motor_r.flush(); break;
        case 2: if (_f_gs)      _f_gs.flush();      break;
    }
    _flush_turn = (uint8_t)((_flush_turn + 1) % 3);
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
