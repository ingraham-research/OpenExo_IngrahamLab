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
SPLINEALT_PLANTAR_RISE_TIME = 4     #Percent gait spent rising from zero up to peak plantarflexion
SPLINEALT_PLANTAR_DWELL_TIME = 5    #0 = no plateau. Zero is fine here; zero RISE or FALL is not (see below)
SPLINEALT_PLANTAR_FALL_TIME = 6     #Percent gait spent falling from peak plantarflexion back down to zero
SPLINEALT_DORSI_RISE_TIME = 7       #Percent gait spent rising from zero up to peak dorsiflexion
SPLINEALT_DORSI_DWELL_TIME = 8      #Same meaning as the plantar dwell - 0 = no plateau, which is the normal case
SPLINEALT_DORSI_FALL_TIME = 9       #Percent gait spent falling from peak dorsiflexion back down to zero
SPLINEALT_TORQUE_SCALE = 10         #0-100 percent, applied to BOTH lobes. This is the machine action
SPLINEALT_SIM_GAIT = 11             #Flag to fake the gait cycle, for benchtop testing with nobody walking
SPLINEALT_USE_PERCENT_GAIT = 12     #0 = percent stance (legacy), 1 = percent gait. Everything above assumes 1
SPLINEALT_USE_PID = 13              #Flag for closed loop torque control against the torque sensor
SPLINEALT_P_GAIN = 14               #The gains for that loop. splineAlt.csv ships 3 / 0 / 0.01, and we never write them
SPLINEALT_I_GAIN = 15
SPLINEALT_D_GAIN = 16

#"See below" on the dwell line: the Teensy builds each lobe as (peak - rise, 0) (peak, amp) (peak + dwell, amp)
#(peak + dwell + fall, 0), and PCHIP rejects two nodes sharing an x. A zero DWELL simply collapses the lobe to
#three nodes, which is the normal no-plateau case. A zero RISE or FALL instead duplicates an x, the whole
#profile is thrown away and the controller commands ZERO torque - deliberately, because the alternative would
#have been full peak torque from 0 percent gait onward. So never write 0 to a rise or a fall time.
#
#Note this class only ever writes indices 0, 1, 2, 3 and 10. Everything else is left at whatever splineAlt.csv
#loaded, so the gains and the flags stay a GUI/SD card decision rather than something the backend can move.



class SplineAlt_action_map:
    """This class holds the resolved parameter addresses for the splineAlt controller on both ankles and
    applies machine actions to them. It is the only place that knows a "machine action" is a torque
    percentage and that a torque percentage is splineAlt's TorqScale.

    Inputs:
    exo_link: a connected OpenExoLink instance
    joints: the two joint names (or ids) to drive. Both ankles by default
    controller: the controller name to drive. Must be one the exo advertised
    torque_percent_max: hard ceiling applied to torque percentage BEFORE it is sent. The firmware bounds-checks
        too, and here we just make sure it doesnt go beyond 100.
    body_mass_standard: the body mass in kg that the peak torque magnitudes correspond to. NO DEFAULT
        on purpose - set by main_external_control
    verbose: if 1, print each action as it is applied
    """

    def __init__(self, exo_link, body_mass_standard, joints=("Ankle(L)", "Ankle(R)"),
                 controller="splineAlt", torque_percentage_max=100.0, verbose=1):
        self.exo_link = exo_link                        #The open link to the GUI. We never talk to the exo any other way
        self.joints = list(joints)                      #Kept as a list so it can be zipped alongside the address lists below
        self.controller = controller                    #Only kept for messages; the addresses below are already resolved
        self.torque_percentage_max = torque_percentage_max        #Our own ceiling on the torque percentage (machine action for game theory), checked before every write
        self.body_mass_standard = body_mass_standard    #In kg. The body mass the peak torque magnitudes are quoted for, same idea as the hip exo
        self.verbose = verbose                          #If 1, every action prints as it is applied

        self.last_applied_action = None     #The last torque percentage we successfully wrote to BOTH sides

        #Resolve every address once, up front, so a typo in a name fails here at setup rather than
        #half way through an experiment. resolve() turns (joint name, controller name, parameter index) into
        #the numeric address the remote service expects, and raises if any of the three is not one the exo
        #advertised. Each list below holds one address PER JOINT, in the same order as self.joints, so
        #writing to both legs is just a zip over the pair
        self._torque_scale_addr = [exo_link.resolve(j, controller, SPLINEALT_TORQUE_SCALE) for j in self.joints]           #The machine action itself, written every loop
        self._plantar_nm_addr = [exo_link.resolve(j, controller, SPLINEALT_MAX_PLANTAR_TORQUE) for j in self.joints]       #Session setup only, from here on
        self._dorsi_nm_addr = [exo_link.resolve(j, controller, SPLINEALT_MAX_DORSI_TORQUE) for j in self.joints]
        self._peak_time_addr = {    #Keyed by lobe name, so apply_peak_timing can be told "plantar" or "dorsi" instead of an index
            "plantar": [exo_link.resolve(j, controller, SPLINEALT_PEAK_PLANTAR_TIME) for j in self.joints],
            "dorsi": [exo_link.resolve(j, controller, SPLINEALT_PEAK_DORSI_TIME) for j in self.joints],
        }

        if self.verbose:    #Print what we resolved, so a wrong joint or controller name shows up here rather than mid-trial
            print(f"Action map ready: {controller} on {self.joints}, machine action capped at {torque_percentage_max}%")

    ############################### Session setup #############################################

    def engage_controller_safely(self):
        '''Switch both ankles onto splineAlt with the torque scale at ZERO, before anything else.
            This has to be the first write, because writing ANY parameter to a controller that is not the active one switches the joint to that controller AND loads that
            controller's defaults from the SD card. 

            The default TorqScale in splineAlt.csv is most likely non-zero (95 as of this writing), so if the first write were, say, the peak torque magnitude, the joint would switch over and immediately begin
            assisting at 95% - before we have set the participant's torque magnitudes, and before the backend has chosen anything.

            Writing TorqScale = 0 first means the controller comes up in transparency mode.'''
        
        if self.verbose:
            print("Engaging splineAlt at zero torque scale (transparency) before anything else...")
        for addr, joint in zip(self._torque_scale_addr, self.joints):   #Walk every joint we were given, one write each
            self.exo_link.set_param_confirmed(addr, 0.0, label=f"{joint} TorqScale")
        self.last_applied_action = 0.0  #This should read as "we just deliberately applied zero"

    def apply_torque_magnitudes(self, plantar_nm, dorsi_nm, body_mass=None):
        '''Write the peak torque magnitudes, scaled for the participant's body mass. Session setup only -
        the experiment loop never touches this method; apply_machine action and apply_torque_timing changes spline from there on.

        Returns the scaling factor that was applied, so the caller can log it.'''
        scaling = 1.0   #Default to sending the magnitudes exactly as they were passed in, if no body mass was given
        if body_mass is not None:
            scaling = float(body_mass) / self.body_mass_standard    #The same body_mass_scaling_factor the hip exo applies to its OP

        scaled_plantar = plantar_nm * scaling   #What actually goes on the wire. The caller's own numbers are kept unscaled for the log line below
        scaled_dorsi = dorsi_nm * scaling
        if self.verbose:
            print(f"Torque magnitudes scaled by {scaling:.2f} for body mass: "
                  f"plantar {plantar_nm} -> {scaled_plantar:.2f} Nm, dorsi {dorsi_nm} -> {scaled_dorsi:.2f} Nm")

        #Now, we set the parameters directly in this method
        #One loop per parameter rather than one loop doing both, so that if a write fails partway through we
        #know exactly which parameter we were on. Each loop is just "every joint, same value"
        for addr, joint in zip(self._plantar_nm_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, scaled_plantar, label=f"{joint} PlantarNm")
        for addr, joint in zip(self._dorsi_nm_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, scaled_dorsi, label=f"{joint} DorsiNm")
        return scaling  #Handed back purely so the caller can record the factor its magnitudes actually got

    ############################### The experiment loop action #############################################

    def apply_torque_percentage(self, _scale):
        '''Apply a torque scaling percentage - to both ankles. Returns the value actually applied after clamping.

            If either side fails to acknowledge, OpenExoLink raises and the caller must decide what to do. 
            Not that a silence isn't guaranteed to be a failure to implement, but likely something went wrong'''
        _scale_clamped = min(max(float(_scale), 0.0), self.torque_percentage_max)  #Floor at 0 as well as cap at m_max - a negative scale would flip both lobes
        if _scale_clamped != float(_scale) and self.verbose:      #Only say something when we actually changed the backend's number
            print(f"  Machine action {_scale:.2f}% clamped to {_scale_clamped:.2f}% by our own safety limit")

        for addr, joint in zip(self._torque_scale_addr, self.joints):   #Every joint gets the same scale, so both legs assist identically
            self.exo_link.set_param_confirmed(addr, _scale_clamped, label=f"{joint} TorqScale")

        #Only updated once BOTH sides have acknowledged. If the second write raised we never reach this line, and
        #last_applied_action keeps pointing at the last action we know was actually on both legs
        self.last_applied_action = _scale_clamped
        return _scale_clamped    #The caller should log THIS, not its own scale - the two differ whenever the clamp fired

    def apply_peak_timing(self, lobe, percent_gait):
        '''Move the peak of one lobe ("plantar" or "dorsi") to a new percent of the gait cycle, on bothsides. 
            This is what a UDP timing value drives, and it is the ankle equivalent of the hip exo's create_flexion_only_torque_profile(udp_timing_value).

            Unlike the hip exo, where a new timing meant rebuilding the whole spline in Python, here it is one parameter write per leg - the Teensy rebuilds its own nodes from it.'''
        if lobe not in self._peak_time_addr:    #Catch a mistyped lobe name here, before we have written anything to the exo
            raise ValueError(f"lobe must be 'plantar' or 'dorsi', got {lobe!r}")
        for addr, joint in zip(self._peak_time_addr[lobe], self.joints):    #Every joint, same new peak time
            self.exo_link.set_param_confirmed(addr, float(percent_gait), label=f"{joint} {lobe} peak time")
        return float(percent_gait)  #Hand back what we actually wrote, so the caller logs the value the exo has rather than the one it asked for

    def park_to_transparency(self):
        '''Drop both sides to zero torque scale. Used on shutdown and whenever something has gone wrong.
            Best effort only - if the link is already dead this cannot succeed.
            Returns 1 if both sides acknowledged the park, 0 if it could not be done.'''
        try:
            self.apply_torque_percentage(0.0)  #Goes through the normal path, so the park gets acknowledged like any other action
            print("Parked both ankles to zero torque scale (transparency)")
            return 1
        except Exception as e:  #Catch everything on purpose - this runs in shutdown and error paths, so it must never raise on its own
            print(f"WARNING: could not park the exo to transparency: {e}")
            print("The exo is still running whatever it had last. Stop the trial from the GUI.")
            return 0
