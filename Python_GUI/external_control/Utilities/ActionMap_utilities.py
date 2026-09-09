#This utility turns an abstract "machine action" from an optimization backend into the concrete controller parameter writes that implement it on the exo.
#Note that it's main significance is that GUI (at least the remote service we wrote) takes commands by index, so it's important to match what we want to which index we actually write to
#Index, such as joint_id, controller_id, param_index - these are mapped and held in this class.
#Therefore, modifications to the actual settings are applied to this class, as well

#For the game theory backend with self-paced treadmill input, the machine action is a TORQUE PERCENTAGE (0-100), exactly as in the hip exo's main_control_code:
#    new_torque_profile = [(t, val * new_torque_percentage * body_mass_scaling_factor / 100) for t, val in OP]

#On the ankle exo the whole profile lives on the Teensy, so the equivalent is a single parameter: splineAlt's TorqScale, which scales both torque lobes at node construction time.

#splineAlt parameter indices. Source of truth is ExoCode/src/ControllerData.h (namespace
#controller_defs::spline_alt), mirrored by the column order in SDCard/ankleControllers/splineAlt.csv
SPLINEALT_MAX_PLANTAR_TORQUE = 0    #Peak plantarflexion magnitude in Nm, entered POSITIVE, applied negative
SPLINEALT_MAX_DORSI_TORQUE = 1      #Peak dorsiflexion magnitude in Nm
SPLINEALT_PEAK_PLANTAR_TIME = 2     #Percent gait at which plantarflexion first reaches peak
SPLINEALT_PEAK_DORSI_TIME = 3       #Percent gait at which dorsiflexion first reaches peak
SPLINEALT_PLANTAR_RISE_TIME = 4
SPLINEALT_PLANTAR_DWELL_TIME = 5    #0 = no plateau. Zero is fine here; zero RISE or FALL is not (see below)
SPLINEALT_PLANTAR_FALL_TIME = 6
SPLINEALT_DORSI_RISE_TIME = 7
SPLINEALT_DORSI_DWELL_TIME = 8
SPLINEALT_DORSI_FALL_TIME = 9
SPLINEALT_TORQUE_SCALE = 10         #0-100 percent, applied to BOTH lobes. This is the machine action
SPLINEALT_SIM_GAIT = 11
SPLINEALT_USE_PERCENT_GAIT = 12
SPLINEALT_USE_PID = 13
SPLINEALT_P_GAIN = 14
SPLINEALT_I_GAIN = 15
SPLINEALT_D_GAIN = 16



class SplineAlt_action_map:
    """This class holds the resolved parameter addresses for the splineAlt controller on both ankles and
    applies machine actions to them. It is the only place that knows a "machine action" is a torque
    percentage and that a torque percentage is splineAlt's TorqScale.

    Inputs:
    exo_link: a connected ExoLink instance
    joints: the two joint names (or ids) to drive. Both ankles by default
    controller: the controller name to drive. Must be one the exo advertised
    m_max: hard ceiling applied to every machine action BEFORE it is sent. The firmware bounds-checks
        too, but this is our own safety limit and should start well below 100
    body_mass_standard: the body mass in kg that the peak torque magnitudes correspond to. NO DEFAULT
        on purpose - set by main_external_control
    verbose: if 1, print each action as it is applied
    """

    def __init__(self, exo_link, body_mass_standard, joints=("Ankle(L)", "Ankle(R)"),
                 controller="splineAlt", m_max=100.0, verbose=1):
        self.exo_link = exo_link
        self.joints = list(joints)
        self.controller = controller
        self.m_max = m_max
        self.body_mass_standard = body_mass_standard
        self.verbose = verbose

        self.last_applied_action = None     #The last torque percentage we successfully wrote to BOTH sides

        #Resolve every address once, up front, so a typo in a name fails here at setup rather than
        #half way through an experiment
        self._torque_scale_addr = [exo_link.resolve(j, controller, SPLINEALT_TORQUE_SCALE) for j in self.joints]
        self._plantar_nm_addr = [exo_link.resolve(j, controller, SPLINEALT_MAX_PLANTAR_TORQUE) for j in self.joints]
        self._dorsi_nm_addr = [exo_link.resolve(j, controller, SPLINEALT_MAX_DORSI_TORQUE) for j in self.joints]
        self._peak_time_addr = {
            "plantar": [exo_link.resolve(j, controller, SPLINEALT_PEAK_PLANTAR_TIME) for j in self.joints],
            "dorsi": [exo_link.resolve(j, controller, SPLINEALT_PEAK_DORSI_TIME) for j in self.joints],
        }

        if self.verbose:
            print(f"Action map ready: {controller} on {self.joints}, machine action capped at {m_max}%")

    ############################### Session setup #############################################

    def engage_controller_safely(self):
        '''Switch both ankles onto splineAlt with the torque scale at ZERO, before anything else.

        This has to be the first write, and the reason is a trap worth spelling out. Writing ANY parameter
        to a controller that is not the active one switches the joint to that controller AND loads that
        controller's defaults from the SD card. The default TorqScale in splineAlt.csv is 95, so if the
        first write were, say, the peak torque magnitude, the joint would switch over and immediately begin
        assisting at 95% - before we have set the participant's torque magnitudes, and before the backend
        has chosen anything.

        Writing TorqScale = 0 first means the controller comes up in transparency mode. That is a real mode,
        not a hack: at scale 0 every node collapses to zero and the gain scheduler holds the tuned
        zero-torque gains for the whole stride (see ExoCode/src/Controller.cpp, SplineAlt::_build_nodes).'''
        if self.verbose:
            print("Engaging splineAlt at zero torque scale (transparency) before anything else...")
        for addr, joint in zip(self._torque_scale_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, 0.0, label=f"{joint} TorqScale")
        self.last_applied_action = 0.0

    def apply_torque_magnitudes(self, plantar_nm, dorsi_nm, body_mass=None):
        '''Write the peak torque magnitudes, scaled for the participant's body mass. Session setup only -
        the experiment loop never touches these, it only moves TorqScale.

        Returns the scaling factor that was applied, so the caller can log it.'''
        scaling = 1.0
        if body_mass is not None:
            scaling = float(body_mass) / self.body_mass_standard

        scaled_plantar = plantar_nm * scaling
        scaled_dorsi = dorsi_nm * scaling
        if self.verbose:
            print(f"Torque magnitudes scaled by {scaling:.2f} for body mass: "
                  f"plantar {plantar_nm} -> {scaled_plantar:.2f} Nm, dorsi {dorsi_nm} -> {scaled_dorsi:.2f} Nm")

        for addr, joint in zip(self._plantar_nm_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, scaled_plantar, label=f"{joint} PlantarNm")
        for addr, joint in zip(self._dorsi_nm_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, scaled_dorsi, label=f"{joint} DorsiNm")
        return scaling

    def apply_peak_timing(self, lobe, percent_gait):
        '''Move the peak of one lobe ("plantar" or "dorsi") to a new percent of the gait cycle, on both
        sides. This is what a UDP timing value drives, and it is the ankle equivalent of the hip exo's
        create_flexion_only_torque_profile(udp_timing_value).

        Unlike the hip exo, where a new timing meant rebuilding the whole spline in Python, here it is one
        parameter write per leg - the Teensy rebuilds its own nodes from it.'''
        if lobe not in self._peak_time_addr:
            raise ValueError(f"lobe must be 'plantar' or 'dorsi', got {lobe!r}")
        for addr, joint in zip(self._peak_time_addr[lobe], self.joints):
            self.exo_link.set_param_confirmed(addr, float(percent_gait), label=f"{joint} {lobe} peak time")
        return float(percent_gait)

    ############################### The experiment loop action #############################################

    def apply_machine_action(self, m):
        '''Apply one machine action - a torque percentage - to both ankles. Returns the value actually
        applied after clamping.

        If either side fails to acknowledge, ExoLink raises and the caller must decide what to do. We do
        not swallow it here: a half-applied action means the two legs are assisting differently, which is
        worse than stopping.'''
        m_clamped = min(max(float(m), 0.0), self.m_max)
        if m_clamped != float(m) and self.verbose:
            print(f"  Machine action {m:.2f}% clamped to {m_clamped:.2f}% by our own safety limit")

        for addr, joint in zip(self._torque_scale_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, m_clamped, label=f"{joint} TorqScale")

        self.last_applied_action = m_clamped
        return m_clamped

    def park_to_transparency(self):
        '''Drop both sides to zero torque scale. Used on shutdown and whenever something has gone wrong.

        Best effort only - if the link is already dead this cannot succeed, and saying so plainly is more
        useful than pretending the exo was parked.'''
        try:
            self.apply_machine_action(0.0)
            print("Parked both ankles to zero torque scale (transparency)")
            return 1
        except Exception as e:
            print(f"WARNING: could not park the exo to transparency: {e}")
            print("         The exo is still running whatever it had last. Stop the trial from the GUI.")
            return 0
