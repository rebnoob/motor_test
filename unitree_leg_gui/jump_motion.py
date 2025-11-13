# jump_motion.py
import time

class JumpMotion:
    """
    Joint-space jump:
      hold -> crouch -> explode -> done
    We don't move IK targets here. We just say:
      - which phase we're in
      - what joint offsets to add (pitch/knee)
    The GUI/control loop applies those offsets on top of its normal targets.
    """
    def __init__(self, on_phase_change=None):
        self.on_phase_change = on_phase_change
        self.running = False
        self.current_phase = None

        # GUI-tunable
        self.hold_s    = 0.25
        self.crouch_s  = 0.25
        self.explode_s = 0.18

        # joint offsets in *output-side radians*
        # positive/negative here depends on your leg’s “down” direction;
        # we can start with “crouch = bend more” -> negative pitch, positive knee
        self.crouch_pitch_rad = -0.20   # ~ -11.5 deg
        self.crouch_knee_rad  =  0.25   # ~ 14 deg
        self.explode_pitch_rad = 0.35   # leg extends forward/down
        self.explode_knee_rad  = -0.40  # leg straightens

        # current offsets we want the controller to add
        self.offset_pitch = 0.0
        self.offset_knee  = 0.0

    def start(self):
        if self.running:
            return
        self.running = True
        self.offset_pitch = 0.0
        self.offset_knee  = 0.0

    def stop(self):
        self.running = False
        self.offset_pitch = 0.0
        self.offset_knee  = 0.0
        self._emit_phase(None)

    def _emit_phase(self, phase):
        self.current_phase = phase
        if self.on_phase_change:
            self.on_phase_change(phase)

    def run(self):
        t0 = time.monotonic()
        while self.running:
            t = time.monotonic() - t0

            hold    = max(float(self.hold_s),    0.05)
            crouch  = max(float(self.crouch_s),  0.05)
            explode = max(float(self.explode_s), 0.05)

            T_hold    = hold
            T_crouch  = T_hold + crouch
            T_explode = T_crouch + explode

            if t <= T_hold:
                phase = "hold"
                # no offsets
                self.offset_pitch = 0.0
                self.offset_knee  = 0.0

            elif t <= T_crouch:
                phase = "crouch"
                alpha = (t - T_hold) / crouch
                # ramp into crouch offsets
                self.offset_pitch = alpha * self.crouch_pitch_rad
                self.offset_knee  = alpha * self.crouch_knee_rad

            elif t <= T_explode:
                phase = "explode"
                alpha = (t - T_crouch) / explode
                # go from crouch offsets toward explode offsets
                pitch_start = self.crouch_pitch_rad
                knee_start  = self.crouch_knee_rad
                pitch_target = self.explode_pitch_rad
                knee_target  = self.explode_knee_rad
                self.offset_pitch = pitch_start + alpha * (pitch_target - pitch_start)
                self.offset_knee  = knee_start  + alpha * (knee_target  - knee_start)

            else:
                break

            if phase != self.current_phase:
                self._emit_phase(phase)

            time.sleep(0.01)

        # done
        self.running = False
        self.offset_pitch = 0.0
        self.offset_knee  = 0.0
        self._emit_phase(None)
