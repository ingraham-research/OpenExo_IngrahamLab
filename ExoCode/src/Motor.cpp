/*
 * P. Stegall Jan. 2022
*/

#include "Arduino.h" 

#include "Motor.h"
#include "CAN.h"
#include "ErrorManager.h"
#include "error_codes.h"
#include "Logger.h"
#include "ErrorReporter.h"
#include "error_codes.h"
//#define MOTOR_DEBUG           //Uncomment if you want to print debug statments to the serial monitor

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 


_Motor::_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin)
{
    _id = id;
    _is_left = ((uint8_t)this->_id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    _data = exo_data;
    _enable_pin = enable_pin;
    _prev_motor_enabled = false; 
    _prev_on_state = false;
    
    #ifdef MOTOR_DEBUG
        logger::print("_Motor::_Motor : _enable_pin = ");
        logger::print(_enable_pin);
        logger::print("\n");
    #endif
    
    pinMode(_enable_pin, OUTPUT);
    
    //Set _motor_data to point to the data specific to this motor.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.hip.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.hip.motor);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.knee.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.knee.motor);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.ankle.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.ankle.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::elbow:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.elbow.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.elbow.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::arm_1:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.arm_1.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.arm_1.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::arm_2:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.arm_2.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.arm_2.motor);
            }
            break;
    }

    #ifdef MOTOR_DEBUG
        logger::println("_Motor::_Motor : Leaving Constructor");
    #endif

};

bool _Motor::get_is_left() 
{
    return _is_left;
};

config_defs::joint_id _Motor::get_id()
{
    return _id;
};

/*
 * Constructor for the CAN Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the CAN (e.g. TMotor) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model (e.g. AK60), which may have specific torque constants etc.
 */
_CANMotor::_CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) //Constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{
    _KP_MIN = 0.0f;
    _KP_MAX = 500.0f;
    _KD_MIN = 0.0f;
    _KD_MAX = 5.0f;
    _P_MAX = 12.5f;

    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));
    j_data->motor.kt = this->get_Kt();

    _enable_response = false;

    #ifdef MOTOR_DEBUG
        logger::println("_CANMotor::_CANMotor : Leaving Constructor");
    #endif
};

void _CANMotor::transaction(float torque)
{
    //Send data and read response 
    send_data(torque);
    read_data();
    check_response();
};

void _CANMotor::read_data()
{
    // Read on EVERY cycle, enabled or not (this used to be `if (_motor_data->enabled)`).
    //
    // Two reasons, both of which became load-bearing once send_data() started transmitting
    // zero-torque frames while disabled:
    //   1. The motor replies to every command frame. If we transmit but never read, those
    //      replies pile up in the single shared FlexCAN RX queue. CAN::read() is one
    //      destructive pop routed by QUEUE POSITION rather than by CAN id, so a backlog
    //      permanently offsets the left/right alternation and each motor starts consuming
    //      (and discarding) the other's frames. Draining every cycle prevents that.
    //   2. Gating the read on `enabled` froze _motor_data->i the moment the motor was
    //      disabled -- and that frozen value is exactly what check_response() measures the
    //      variance of. The check meant to detect "motor stopped responding" had its input
    //      disabled by the very flag it reacts to, so its re-enable was guaranteed to fire
    //      eventually. Reading unconditionally keeps the variance signal real.
    {
        CAN* can = can->getInstance();
        int direction_modifier = _motor_data->flip_direction ? -1 : 1;

        CAN_message_t msg = can->read();

        // Determine if the motor type is AK60v3 (extended, new format) or old AK (standard, old format)
		// Background: AK60v3 motors employ a different communication protocol and are handled differently in this context.
        bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);
    
        // When the motor type is AK60v3
        if (is_ak60v3) {
            if (msg.len == 0 || !msg.flags.extended) {
                return;
            }
            // AK60v3: NEW message format
            if ((msg.id & 0xFF) == uint8_t(_motor_data->id))
            {
                // AK60v3 (AK60-6 V3.0) sends the CubeMars CAN status-upload frame (AK Series Manual
                // V3.0.0 sec 4.3.1), NOT the MIT unsigned-with-offset encoding: signed int16 fields with
                // fixed scales -- [0..1]=position (0.1 deg/count), [2..3]=speed (10 ERPM/count, electrical),
                // [4..5]=current (0.01 A/count), [6]=temp (int8), [7]=error. Verified against the manual's
                // reference decode, the lab's working V3 Python driver, and log correlation.
                // motor.p/v/i are expressed at the actuator output shaft: the internal 6:1 is folded into
                // Kt and the external 4.5:1 joint gearing is applied in Joint.cpp. Speed is ELECTRICAL, so
                // ERPM -> output rad/s divides by pole_pairs(14) * internal_reduction(6).
                int16_t p_raw = (int16_t)((msg.buf[0] << 8) | msg.buf[1]);
                int16_t v_raw = (int16_t)((msg.buf[2] << 8) | msg.buf[3]);
                int16_t i_raw = (int16_t)((msg.buf[4] << 8) | msg.buf[5]);
                _motor_data->p = direction_modifier * (p_raw * 0.1f) * PI / 180.0f;
                _motor_data->v = direction_modifier * (v_raw * 10.0f) / (14.0f * 6.0f) * (2.0f * PI / 60.0f);
                _motor_data->i = direction_modifier * (i_raw * 0.01f);
                #ifdef MOTOR_DEBUG
                    logger::print("_CANMotor::read_data():Got data-");
                    logger::print("ID:" + String(uint32_t(_motor_data->id)) + ",");
                    logger::print("P:"+String(_motor_data->p) + ",V:" + String(_motor_data->v) + ",I:" + String(_motor_data->i));
                    logger::print("\n");
                #endif
                _motor_data->timeout_count = 0;
            }
        }
		//When the motor type is a CAN motor other than the AK60v3.
		else {
            if (msg.len == 0 || msg.flags.extended) {
                return;
            }
            // Old AK: OLD message format
            if (msg.buf[0] == uint32_t(_motor_data->id))
            {
                uint32_t p_int = (msg.buf[1] << 8) | msg.buf[2];
                uint32_t v_int = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
                uint32_t i_int = ((msg.buf[4] & 0xF) << 8) | msg.buf[5];
                _motor_data->p = direction_modifier * _uint_to_float(p_int, -_P_MAX, _P_MAX, 16);
                _motor_data->v = direction_modifier * _uint_to_float(v_int, -_V_MAX, _V_MAX, 12);
                _motor_data->i = direction_modifier * _uint_to_float(i_int, -_I_MAX, _I_MAX, 12);
                #ifdef MOTOR_DEBUG
                    logger::print("_CANMotor::read_data():Got data-");
                    logger::print("ID:" + String(uint32_t(_motor_data->id)) + ",");
                    logger::print("P:"+String(_motor_data->p) + ",V:" + String(_motor_data->v) + ",I:" + String(_motor_data->i));
                    logger::print("\n");
                #endif
                _motor_data->timeout_count = 0;
            }
        }
    }
    return;
};

void _CANMotor::send_data(float torque)
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif

    // ================= HARD OUTPUT CLAMP - LAST LINE OF DEFENCE =================
    // This is deliberately the FINAL gate before the CAN frame is built, so it sits AFTER
    // everything: feed-forward, PID, gain scheduling, every controller, every joint. The only
    // pre-existing limits were on controller-internal feed-forward terms, so a PID output could
    // reach the motor unbounded. Nothing this exo does should ever ask for more than
    // MAX_JOINT_TORQUE_NM at the joint; if something computes more, it is a fault, not a command.
    //
    // `torque` here is at the MOTOR shaft. The joint sees torque * gearing (4.5 for our ankles),
    // so the motor-side limit is MAX_JOINT_TORQUE_NM / gearing.
    //
    // Non-finite values are rejected separately because constrain() is a MACRO: every comparison
    // against NaN is false, so NaN passes through unchanged. It then reaches _float_to_uint, and
    // (unsigned int)NaN saturates to 0 on Cortex-M7 -- which the motor decodes as -I_MAX, i.e.
    // FULL NEGATIVE TORQUE. A NaN anywhere upstream would otherwise become a maximum-torque command.
    {
        const float gearing = (_motor_data->gearing > 0.0f) ? _motor_data->gearing : 1.0f;
        const float max_motor_torque = (float)MAX_JOINT_TORQUE_NM / gearing;

        // Reporting note: logger::print() is gated on `level <= logging::level`, and
        // logging::level is Release(0), so ONLY LogLevel::Release messages survive -- a plain
        // logger::print() here would be silently discarded. A clamp is a safety event and must be
        // visible, so use Serial directly. Rate-limited to 1 line/second because a stuck fault
        // would otherwise print at 1000 lines/second and stall the control loop; the counters make
        // sure a burst that lasts <1 s is still reported.
        static uint32_t clamp_count = 0;
        static uint32_t nonfinite_count = 0;
        static uint32_t last_report_ms = 0;
        bool tripped = false;

        if (isnan(torque) || isinf(torque))
        {
            nonfinite_count++;
            torque = 0.0f;
            tripped = true;
        }
        else if (torque > max_motor_torque || torque < -max_motor_torque)
        {
            clamp_count++;
            if (millis() - last_report_ms > 1000)
            {
                last_report_ms = millis();
                Serial.print("TORQUE CLAMP: motor ");
                Serial.print(uint32_t(_motor_data->id));
                Serial.print(" asked for ");
                Serial.print(torque * gearing, 1);
                Serial.print(" Nm at the joint, limit ");
                Serial.print((float)MAX_JOINT_TORQUE_NM, 1);
                Serial.print(" Nm. clamped=");
                Serial.print(clamp_count);
                Serial.print(" nonfinite=");
                Serial.println(nonfinite_count);
            }
            torque = (torque > 0.0f) ? max_motor_torque : -max_motor_torque;
            tripped = false;   // already reported above
        }

        if (tripped && (millis() - last_report_ms > 1000))
        {
            last_report_ms = millis();
            Serial.print("TORQUE CLAMP: motor ");
            Serial.print(uint32_t(_motor_data->id));
            Serial.print(" NON-FINITE command -> 0. clamped=");
            Serial.print(clamp_count);
            Serial.print(" nonfinite=");
            Serial.println(nonfinite_count);
        }
    }
    // ============================================================================

    int direction_modifier = _motor_data->flip_direction ? -1 : 1;

    // t_ff and last_command are NOT set here. They are set in the transmit branches at the bottom
    // of this function so they record WHAT WENT ON THE WIRE, not what the controller computed.
    // Previously they were assigned here, unconditionally -- so when the motor was disabled we put
    // a ZERO frame on the bus but recorded the controller's (possibly large) value. Any log built
    // on them then showed a big "commanded torque" for a cycle that actually commanded zero, which
    // makes it impossible to tell "faulty command followed by zeros" from "faulty command followed
    // by nothing" -- exactly the distinction we need at End Trial.
    const float current = torque / get_Kt();

    float p_sat = constrain(direction_modifier * _motor_data->p_des, -_P_MAX, _P_MAX);
    float v_sat = constrain(direction_modifier * _motor_data->v_des, -_V_MAX, _V_MAX);
    float kp_sat = constrain(_motor_data->kp, _KP_MIN, _KP_MAX);
    float kd_sat = constrain(_motor_data->kd, _KD_MIN, _KD_MAX);
    float i_sat = constrain(direction_modifier * current, -_I_MAX, _I_MAX);
    uint32_t p_int = _float_to_uint(p_sat, -_P_MAX, _P_MAX, 16);
    uint32_t v_int = _float_to_uint(v_sat, -_V_MAX, _V_MAX, 12);
    uint32_t kp_int = _float_to_uint(kp_sat, _KP_MIN, _KP_MAX, 12);
    uint32_t kd_int = _float_to_uint(kd_sat, _KD_MIN, _KD_MAX, 12);
    uint32_t i_int = _float_to_uint(i_sat, -_I_MAX, _I_MAX, 12);

    CAN_message_t msg;
    
    // Determine if this is an AK60v3 (extended, new format) or old AK (standard, old format)
    bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

    if (is_ak60v3) {
        // AK60v3: Extended CAN, NEW format
        msg.flags.extended = 1;
        msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
        // NEW format
        msg.buf[0] = kp_int >> 4;
        msg.buf[1] = ((kp_int&0xF) << 4) | (kd_int >> 8);
        msg.buf[2] = (kd_int & 0xFF);
        msg.buf[3] = p_int >> 8;
        msg.buf[4] = p_int & 0xFF;
        msg.buf[5] = v_int >> 4;
        msg.buf[6] = ((v_int & 0xF) << 4) | (i_int >> 8);
        msg.buf[7] = i_int & 0xFF;
    } else {
        // Old AK: Standard CAN, OLD format
        msg.flags.extended = 0;
        msg.id = (uint32_t)_motor_data->id;
        // OLD format
        msg.buf[0] = p_int >> 8;
        msg.buf[1] = p_int & 0xFF;
        msg.buf[2] = v_int >> 4;
        msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        msg.buf[4] = kp_int & 0xFF;
        msg.buf[5] = kd_int >> 4;
        msg.buf[6] = ((kd_int & 0xF) << 4) | (i_int >> 8);
        msg.buf[7] = i_int & 0xFF;
    }
    logger::print("_CANMotor::send_data::t_sat:: ");
    logger::print(torque);
    logger::print("\n");

    CAN* can = can->getInstance();

    if (_motor_data->enabled)
    {
        //Set data in motor
        can->send(msg);
        _prev_motor_enabled = true;
        //Record what actually went on the wire (see the note where `current` is computed).
        //NB: `t_ff` is a PROTOCOL name (the MIT frame's torque field), not a control one. `torque`
        //here is the FULL post-PID command -- Controller::calc_motor_cmd() returns
        //`torque_cmd + _pid(...)`, and the clamp above has already been applied. It is NOT the
        //controller's feed-forward term (that is controller.ff_setpoint / desired_torque, streamed
        //on RT channels 0/2). See MotorData.h.
        _motor_data->t_ff = torque;          //motor-frame Nm; x gearing = joint Nm
        _motor_data->last_command = i_sat;   //motor-frame amps
    }
    else
    {
        // Motor is disabled - send a zero-torque command EVERY cycle, not just once on the
        // enabled->disabled edge (this used to be `else if (_prev_motor_enabled)`).
        //
        // The AK60v3 holds its last command whenever the frame stream stops, so "disabled"
        // used to mean "freeze whatever was last on the bus" rather than "stop". A single
        // one-shot zero frame only works if nothing transmits after it -- but check_response()
        // can still emit a frame later in the SAME cycle (Motor.cpp, the variance re-enable),
        // and any glitched command frame likewise becomes the held command. Transmitting zero
        // continuously makes the held command always zero: whatever went out on a bad cycle is
        // overwritten ~2 ms later, and the motor is actively commanded to free-spin rather than
        // latching. It also fixes the "ankle freezes on controller change / after End Trial"
        // class of bugs at the root instead of by reset timing.
        //
        // kp=kd=0 and i_ff=0, so the p/v fields are ignored and this is a true free-spin command.
        // Cost: 2 extra frames per control cycle while idle -- the same bus load as an active
        // trial, which is already proven acceptable.
        // See Modification log with claude/End-Trial-Diagnosis-Correction.md
        uint32_t zero_p_int = _float_to_uint(0, -_P_MAX, _P_MAX, 16);
        uint32_t zero_v_int = _float_to_uint(0, -_V_MAX, _V_MAX, 12);
        uint32_t zero_kp_int = _float_to_uint(0, _KP_MIN, _KP_MAX, 12);
        uint32_t zero_kd_int = _float_to_uint(0, _KD_MIN, _KD_MAX, 12);
        uint32_t zero_i_int = _float_to_uint(0, -_I_MAX, _I_MAX, 12);

        if (is_ak60v3) {
            msg.buf[0] = zero_kp_int >> 4;
            msg.buf[1] = ((zero_kp_int&0xF) << 4) | (zero_kd_int >> 8);
            msg.buf[2] = (zero_kd_int & 0xFF);
            msg.buf[3] = zero_p_int >> 8;
            msg.buf[4] = zero_p_int & 0xFF;
            msg.buf[5] = zero_v_int >> 4;
            msg.buf[6] = ((zero_v_int & 0xF) << 4) | (zero_i_int >> 8);
            msg.buf[7] = zero_i_int & 0xFF;
        } else {
            msg.buf[0] = zero_p_int >> 8;
            msg.buf[1] = zero_p_int & 0xFF;
            msg.buf[2] = zero_v_int >> 4;
            msg.buf[3] = ((zero_v_int & 0xF) << 4) | (zero_kp_int >> 8);
            msg.buf[4] = zero_kp_int & 0xFF;
            msg.buf[5] = zero_kd_int >> 4;
            msg.buf[6] = ((zero_kd_int & 0xF) << 4) | (zero_i_int >> 8);
            msg.buf[7] = zero_i_int & 0xFF;
        }

        can->send(msg);
        _prev_motor_enabled = false;
        //A ZERO frame went out, so that is what we recorded as commanded - not the controller's
        //computed value, which was never transmitted.
        _motor_data->t_ff = 0.0f;
        _motor_data->last_command = 0.0f;
    }
    return;
};

void _CANMotor::check_response()
{
    //Only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) ||
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);

    if (_data->user_paused || !active_trial || _data->estop || _error)
    {
        return;
    }

    //Measured current variance should be non-zero
    _measured_current.push(_motor_data->i);

    if (_measured_current.size() > _current_queue_size)
    {
        _measured_current.pop();
        auto pop_vals = utils::online_std_dev(_measured_current);

        // Only attempt to re-enable if the motor is currently disabled
        // Low variance during constant torque is normal and shouldn't trigger re-enable
        if (pop_vals.second < _variance_threshold && !_motor_data->enabled)
        {
            _motor_data->enabled = true;

            // NEVER send the enable frame to an AK60v3 - same guard as Joint.cpp::run_joint().
            // The AK60v3 enables automatically and does not consume the MIT "enter motor mode"
            // special frame. enable() transmits FF FF FF FF FF FF FF FC on ((8<<8)|id), which is
            // the SAME CAN id send_data() uses for torque commands, so the motor unpacks those
            // bytes with the normal MIT field layout: kp=500 (max), kd=5 (max), p_des=+12.5 rad
            // (max), v_des=+48 rad/s (max), i_ff=+10.3 A (max). That is a full-current, max-gain
            // position slam to an unreachable target, so the error never closes and the motor
            // saturates and HOLDS (~51 Nm at the joint) until a new frame arrives. It also skips
            // send_data()'s direction_modifier, so on the flipped side it drives the joint the
            // wrong way. This destroyed the right ankle on 2026-07-23. This branch was the only
            // path that could still reach enable() for an AK60v3, and it is reachable only at
            // End Trial (it needs !enabled while the status is still an active trial).
            // See Modification log with claude/End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md
            if (_motor_data->motor_type != (uint8_t)config_defs::motor::AK60v3)
            {
                enable(true);
            }
        }

    }
};

void _CANMotor::on_off()
{
    if (_data->estop || _error)
    {
        _motor_data->is_on = false;

        // logger::print("_CANMotor::on_off(bool is_on) : E-stop pulled - ");
        // logger::print(uint32_t(_motor_data->id));
        // logger::print("\n");
    }

    if (_prev_on_state != _motor_data->is_on) //If was here to save time, can be removed if making problems, or add overide
    {
        if (_motor_data->is_on)
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_on_state);

            // logger::print("_CANMotor::on_off(bool is_on) : Power on- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
        else 
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_off_state);

            // logger::print("_CANMotor::on_off(bool is_on) : Power off- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
    }
    _prev_on_state = _motor_data->is_on;

    #ifdef HEADLESS
        delay(2000);    //Two second delay between motor's turning on and enabeling, we've run into some issues with enabling while in headless mode if this delay is not present. 
    #endif

};

bool _CANMotor::enable()
{
    return enable(false);
};

bool _CANMotor::enable(bool overide)
{
    // ===== AK60v3: NEVER transmit from here. Guarded at the TOP, not per call site. =====
    // The AK60v3 enables automatically and does not consume the MIT "enter motor mode" special
    // frame. The frame built below is FF FF FF FF FF FF FF FC (or FD) sent on
    // msg.id = ((8 << 8) | id) -- the SAME CAN id send_data() uses for torque -- so the motor
    // unpacks it with the normal field layout: kp=500 (max), kd=5 (max), p_des=+12.5 rad (max),
    // v_des=+48 rad/s (max), t_ff = full scale. That is a max-torque, max-gain position command
    // to an unreachable target: the error never closes, so the motor saturates and HOLDS at
    // T_MAX * gearing = 12.0 * 4.5 = 54.0 Nm at the joint (the AK60-6 unpacks that 12-bit field
    // against +-12.0, manual v3.0.0 sec 4.2 -- NOT against our _I_MAX of 10.3; the older figure of
    // 51.4 Nm assumed I_MAX * Kt * gearing and was low. See Modification log with claude/
    // Motor-Current-Decode-Investigation.md). It also bypasses send_data()'s direction_modifier
    // AND the MAX_JOINT_TORQUE_NM clamp entirely.
    //
    // Guarding here rather than at each caller is deliberate: the per-call-site guards in
    // Joint.cpp::run_joint() and Motor.cpp::check_response() are easy to forget, and
    // Side::disable_motors() (Side.cpp:62) already calls enable(true) on all six motors with no
    // guard at all -- harmless today only because nothing calls it.
    // See Modification log with claude/Fresh-Torque-Path-Safety-Audit.md
    if (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3)
    {
        _prev_motor_enabled = _motor_data->enabled;
        _enable_response = true;   // the AK60v3 is always "enabled"; report success to callers
        return _enable_response;
    }

    #ifdef MOTOR_DEBUG
        //  logger::print(_prev_motor_enabled);
        //  logger::print("\t");
        //  logger::print(_motor_data->enabled);
        //  logger::print("\t");
        //  logger::print(_motor_data->is_on);
        //  logger::print("\n");
    #endif

    //Only change the state and send messages if the enabled state has changed.
    if ((_prev_motor_enabled != _motor_data->enabled) || overide || !_enable_response)
    {
        CAN_message_t msg;

        // Determine if this is an AK60v3 (extended format) or old AK (standard format)
        bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

        // Initialize message format fields
        msg.len = 8;
        if (is_ak60v3) {
            msg.flags.extended = 1;
            msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
        } else {
            msg.flags.extended = 0;
            msg.id = (uint32_t)_motor_data->id;
        }

        msg.buf[0] = 0xFF;
        msg.buf[1] = 0xFF;
        msg.buf[2] = 0xFF;
        msg.buf[3] = 0xFF;
        msg.buf[4] = 0xFF;
        msg.buf[5] = 0xFF;
        msg.buf[6] = 0xFF;

        if (_motor_data->enabled && !_error && !_data->estop)
        {
            msg.buf[7] = 0xFC;
        }
        else
        {
            _enable_response = false;
            msg.buf[7] = 0xFD;
        }

        CAN* can = can->getInstance();
        can->send(msg);
        delayMicroseconds(500);
        read_data();

        if (_motor_data->timeout_count == 0)
        {
            _enable_response = true;
        }
    }

    _prev_motor_enabled = _motor_data->enabled;
    return _enable_response;
};

void _CANMotor::zero()
{
    // ===== AK60v3: NEVER transmit from here. Same reasoning as enable() above. =====
    // This sends FF FF FF FF FF FF FF FE on the torque CAN id, which the AK60v3 decodes as
    // kp=500 / kd=5 / p_des=+12.5 rad / t_ff = one LSB below full scale -- within one LSB of
    // enable()'s frame, and the same ~54 Nm held slam. It bypasses the MAX_JOINT_TORQUE_NM clamp.
    //
    // The AK60v3 has no MIT "set origin" in this protocol, so there is nothing to send. The only
    // caller is Joint.cpp:115 (`if (_joint_data->motor.do_zero) _motor->zero();`), which runs every
    // control cycle. `do_zero` is never set true in the main firmware AND has no initializer in
    // MotorData -- it is false only because ExoData has static storage duration and is therefore
    // zero-initialized. This guard removes the dependence on that.
    // See Modification log with claude/Fresh-Torque-Path-Safety-Audit.md
    if (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3)
    {
        return;
    }

    CAN_message_t msg;

    // Determine if this is an AK60v3 (extended format) or old AK (standard format)
    bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

    // Initialize message format fields
    msg.len = 8;
    if (is_ak60v3) {
        msg.flags.extended = 1;
        msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
    } else {
        msg.flags.extended = 0;
        msg.id = uint32_t(_motor_data->id);
    }

    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;
    CAN* can = can->getInstance();
    can->send(msg);

    read_data();
};

float _CANMotor::get_Kt()
{
    return _Kt;
};

void _CANMotor::set_error()
{
    _error = true;
};

void _CANMotor::set_Kt(float Kt)
{
    _Kt = Kt;
};

void _CANMotor::_handle_read_failure()
{
    // Commented out for AK60v3 integration. 
    //logger::println("CAN Motor - Handle Read Failure", LogLevel::Error);
    //_motor_data->timeout_count++;
};

float _CANMotor::_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12) {
      pgg = (unsigned int) ((x-offset)*4095.0/span); 
    }
    if (bits == 16) {
      pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
};

float _CANMotor::_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits == 12) {
      pgg = ((float)x_int)*span/4095.0 + offset;
    }
    if (bits == 16) {
      pgg = ((float)x_int)*span/65535.0 + offset;
    }
    return pgg;
};

//**************************************
/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60::AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 22.0f;
    _V_MAX = 41.87f;
    
    float kt = 0.068 * 6;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK60::AK60 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60v1_1::AK60v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 13.5f;
    _V_MAX = 23.04f;

    float kt = 0.1725 * 6; //We set KT to 0.1725 * 6 whcih differs from the manufacturer's stated KT, that's because they are wrong (This has been validated mulitple ways). We only have validated for this version as we use open loop at the hip with these, other motors are used with closed loop and thus are corrected in real-time. We recommend validating these KTs if using for open loop. 
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK60v1_1::AK60v1_1 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK80::AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 24.0f;
    _V_MAX = 25.65f;

    float kt = 0.091 * 9;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK80::AK80 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK70::AK70(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 23.2f;
    _V_MAX = 15.5f;
    
    float kt = 0.13 * 10;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK70::AK70 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60v3::AK60v3(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    // !! THESE TWO CONSTANTS DO NOT MATCH THE MOTOR. Do not change either one alone. !!
    //
    // _I_MAX is NOT used as a current limit -- send_data() uses it as the full scale of the MIT
    // t_ff field. 10.3 is the AK60-6 V3.0 datasheet PEAK CURRENT (10.3 A @24V), but the motor
    // unpacks that 12-bit field against +-12.0 (AK Series Module Product Manual v3.0.0 sec 4.2,
    // "Parameter Ranges", AK60-6 column: position +-12.56 rad, speed +-60.0 rad/s, torque
    // +-12.0). CERTAIN, and independent of what unit that +-12.0 carries:
    //
    //     every command we send arrives 12.0/10.3 = 1.165x larger than intended.
    //
    // Side effect to remember: MAX_JOINT_TORQUE_NM = 25 therefore really clamps at 26.2 Nm, and
    // the unguarded enable()/zero() slam is 54.0 Nm.
    //
    // UNRESOLVED: whether that +-12.0 is N.m or IQ amps. The manual contradicts itself -- the
    // sec 4.2 table header says "Motor torque (N.M)", but the sec 4.4.1 command examples label the
    // same field "2A IQ Current" / "4A IQ Current". Those examples cannot settle it (the 2A/4A
    // pair doubles exactly for ANY full scale, so they are circular). Our logs lean N.m: measured
    // current / commanded "amps" runs 1.32-1.62, vs 1.165 predicted if the field is amps.
    // It matters, because it decides the sign of the net error:
    //     field is N.m  -> delivered = requested * (12.0/10.3) * 4.5 / (4.5*1.11) = 1.049 (+5%)
    //     field is amps -> delivered = requested * (12.0/10.3) * 0.81/1.11        = 0.850 (-15%)
    // Either way, do NOT change _I_MAX alone: ->12.0 by itself makes every torque 16.5% smaller.
    // Settle it first with the blocked-joint static test in Modification log with claude/
    // Motor-Current-Decode-Investigation.md, then fix _I_MAX and Kt together.
    _I_MAX = 10.3f;
    _V_MAX = 48.0f;   // manual says +-60.0 rad/s for AK60-6; inert while kd == 0 (it always is)

    // Experimentally determined AK60v3 Kt, including the internal 6:1 gearbox.
    // External joint gearing is handled in Joint.cpp.
    // NB: the datasheet gives 0.135 Nm/A * 6 = 0.81 Nm/A at the output; this 1.11 is ~37% higher.
    // Which one is right is still open and is tangled with the _I_MAX question above -- read that
    // note before touching either value.
    float kt = 0.185f * 6;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK60v3::AK60v3 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK45_36::AK45_36(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 6.5f;
    _V_MAX = 5.44f;

    float kt = 0.127f;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK45_36::AK45_36 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK45_10::AK45_10(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 6.5f;
    _V_MAX = 18.85f;

    float kt = 0.127f;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK45_10::AK45_10 : Leaving Constructor");
#endif
};


/*
 * Constructor for the PWM (Maxon) Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the PWM (e.g. Maxon) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model, which may have specific torque constants etc.
 */
MaxonMotor::MaxonMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) //Constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{
    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));
	
    #ifdef MOTOR_DEBUG
        logger::println("MaxonMotor::MaxonMotor: Leaving Constructor");
    #endif
};

void MaxonMotor::transaction(float torque)
{
    //Send data
    send_data(torque);

    //Only enable the motor when it is an active trial 
    master_switch();

	if (_motor_data->enabled)
	{
		maxon_manager(true); //Monitors for and corrects motor resetting error if the system is operational.
	}
	else
	{
		maxon_manager(false);   //Reset the motor error detection function, in case user pauses device in middle of error event
	}

	// Serial.print("\nRight leg MaxonMotor::transaction(float torque)  |  torque = ");
	// Serial.print(torque);
};

bool MaxonMotor::enable()
{
    return true;    //This function is currently bypassed for this motor at the moment.
};

bool MaxonMotor::enable(bool overide)
{	
	//Only change the state and send messages if the enabled state (used as a master switch for this motor) has changed.
    if ((_prev_motor_enabled != _motor_data->enabled) || overide)
    {
		if (_motor_data->enabled)   //_motor_data->enabled is controlled by the GUI
		{
            //Enable motor
			digitalWrite(_enable_pin,HIGH);         //Relocate in the future
		}

		_enable_response = true;
	}

	if (!overide)                   //When enable(false), send the disable motor command, set the analogWrite resolution, and send 50% PWM command
    {
		_enable_response = false;
		
        //Disable motor, the message after this shouldn't matter as the power is cut, and the send() doesn't send a message if not enabled.
		digitalWrite(_enable_pin,LOW);
		analogWrite(_ctrl_right_pin,_pwm_neutral_val);
		analogWrite(_ctrl_left_pin,_pwm_neutral_val);
    }
	
	if (!_motor_data->enabled)   //_motor_data->enabled is controlled by the GUI
		{
            //Disable motor
			digitalWrite(_enable_pin,LOW);         //Relocate in the future
		}

	_prev_motor_enabled = _motor_data->enabled;

    return _enable_response;
	
    #ifdef MOTOR_DEBUG
        logger::print(_prev_motor_enabled);
        logger::print("\t");
        logger::print(_motor_data->enabled);
        logger::print("\t");
        logger::print(_motor_data->is_on);
        logger::print("\n");
    #endif
};

void MaxonMotor::send_data(float torque) //Always send motor command regardless of the motor "enable" status
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif
	
	int direction_modifier = _motor_data->flip_direction ? -1 : 1; 

	_motor_data->t_ff = torque;
    _motor_data->last_command = torque;
	
	uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) ||
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
   
	if (_data->user_paused || !active_trial || _data->estop)        //Ignores the exo error handler for the moment
    {
        analogWrite(_ctrl_left_pin,_pwm_neutral_val);   //Set 50% PWM (0 current)
		analogWrite(_ctrl_right_pin,_pwm_neutral_val);	//Set 50% PWM (0 current)
    }
    else
    {
		//Constrain the motor pwm command
		uint16_t post_fuse_torque = max(_pwm_l_bound,_pwm_neutral_val+(direction_modifier*torque));    //Set the lowest allowed PWM command
		post_fuse_torque = min(_pwm_u_bound,post_fuse_torque);                              //Set the highest allowed PWM command
		analogWrite((_motor_data->is_left? _ctrl_left_pin : _ctrl_right_pin),post_fuse_torque);	//Send the motor command to the motor driver
	}
};

void MaxonMotor::master_switch()
{
   //Only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);

	if (_data->user_paused || !active_trial || _data->estop)
    {
		pinMode(_err_left_pin, INPUT_PULLUP);
		pinMode(_err_right_pin, INPUT_PULLUP);
		pinMode(_current_left_pin,INPUT);
		pinMode(_current_right_pin,INPUT);
		analogWriteResolution(12);
		analogWriteFrequency(_ctrl_left_pin, 5000);
		analogWriteFrequency(_ctrl_right_pin, 5000);
		
		//_motor_data->enabled = false;
        enable(false);
    }
	else
    {
		//_motor_data->enabled = true;
        enable(true);
	}
};

//Our implementation of the Maxon motor including the ec motor and the Escon 50_8 Motor Controller would occasionally cause 50_8 to enter error mode, with "Over current" being one of the errors.
//To address this issue, we have developed a solution contained in maxon_manager() below. 
void MaxonMotor::maxon_manager(bool manager_active)
{
    //Initialize variables when switch is set to false, run the error detection and rest code when switch is set to true. 
    if (!manager_active)
    {
		//Reset Maxon motor reset utilities
        do_scan4maxon_err_left = true;       
        maxon_counter_active_left = false;
		do_scan4maxon_err_right = true;       
        maxon_counter_active_right = false;
    }
    else
    {
		unsigned long maxon_reset_current_t = millis();
        
		//Scan for left motor error
		if ((do_scan4maxon_err_left) && (!digitalRead(_err_left_pin)))
		{
			do_scan4maxon_err_left = false;          
			maxon_counter_active_left = true;
			zen_millis_left = maxon_reset_current_t;
		}

		//Left motor reset
		if (maxon_counter_active_left) 
		{
			//Two iterations after maxon_counter_actie = true, de-enable motor
			if (maxon_reset_current_t - zen_millis_left >= 2)
			{
				enable(false);
			}

			//Ten iterations after maxon_counter_actie = true, re-enable motor
			if (maxon_reset_current_t - zen_millis_left >= 10)
			{
				enable(true);
			}
			
			//Thirty iterations after maxon_counter_actie = true, start scanning for error again
			if (maxon_reset_current_t - zen_millis_left >= 30)
			{
				do_scan4maxon_err_left = true;
				maxon_counter_active_left = false;                                   
				_motor_data->maxon_plotting_scalar = -1 * _motor_data->maxon_plotting_scalar;
			}
		}

		//Scan for right motor error
		if ((do_scan4maxon_err_right) && (!digitalRead(_err_right_pin)))
		{
			do_scan4maxon_err_right = false;          
			maxon_counter_active_right = true;
			zen_millis_right = maxon_reset_current_t;
		}
		
		//Right motor reset
		if (maxon_counter_active_right) 
		{
			//Two iterations after maxon_counter_actie = true, de-enable motor
			if (maxon_reset_current_t - zen_millis_right >= 2)
			{
				enable(false);
			}

			//Ten iterations after maxon_counter_actie = true, re-enable motor
			if (maxon_reset_current_t - zen_millis_right >= 10)
			{
				enable(true);
			}
			
			//Thirty iterations after maxon_counter_actie = true, start scanning for error again
			if (maxon_reset_current_t - zen_millis_right >= 30)
			{
				do_scan4maxon_err_right = true;
				maxon_counter_active_right = false;                                   
				_motor_data->maxon_plotting_scalar = -1 * _motor_data->maxon_plotting_scalar;
			}
		}
    }
};


#endif
