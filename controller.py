import threading
import time
from timing_utils import FrequencyEstimator


class Controller(threading.Thread):
    def __init__(self, pose_estimator, control_freq=100):
        super().__init__(daemon=True)
        self.pose_estimator = pose_estimator
        self.control_freq = control_freq
        self.stop_event = threading.Event()

        # Control parameters
        self.phase_start = 0.80  # Adjustable via slider
        self.phase_end = 0.20  # Adjustable via slider

        self.max_tilt_deg = 0.5
        self.acceleration_rate = 1.0  # deg/s²
        self.deceleration_rate = 1.0  # deg/s²

        self.current_tilt = 0.0

        # Controller state
        self.last_phase = None
        self.triggered = False

        # Frequency tracking
        self.freq_estimator = FrequencyEstimator(alpha=0.9)

    def update(self, phase, dt):
        if self._is_between_phase(phase, self.phase_start, self.phase_end):
            # In holding zone
            self.current_tilt = self.max_tilt
        elif self._is_after_phase(phase, self.phase_start):
            # Ramp up
            self.current_tilt += self.acceleration_rate * dt
            self.current_tilt = min(self.current_tilt, self.max_tilt)
        elif self._is_before_phase(phase, self.phase_end):
            # Ramp down
            self.current_tilt -= self.deceleration_rate * dt
            self.current_tilt = max(self.current_tilt, 0.0)

    def run(self):

        while not self.pose_estimator.is_ready():
            time.sleep(0.01)

        while not self.pose_estimator.is_finished() and not self.stop_event.is_set():

            state = self.pose_estimator.get_latest_state()
            phase = self.pose_estimator.get_phase()
            direction = self.pose_estimator.get_direction()

            print(
                f"Timestamp: {state['timestamp']:.2f}, COP: ({state['cop_x']:.2f}, {state['cop_y']:.2f}), "
                + f"θ: {state['angle']:.2f}, v: {state['velocity']:.2f}, a: {state['acceleration']:.2f}"
                + f", Phase: {phase:.2f}, Direction: {'Forward' if direction else 'Backward'}"
                + f", Motion State: {state['motion_state']}"
            )

            # Example control logic:
            # Trigger something near 0.8 forward phase
            if direction and phase > 0.8 and not self.triggered:
                print(f"[CONTROL] Triggered at forward phase {phase:.2f}")
                self.triggered = True

            elif not direction and phase > 0.8 and self.triggered:
                print(f"[CONTROL] Triggered at backward phase {phase:.2f}")
                self.triggered = False

            # (You can replace these prints with motor commands, etc.)
            interval = 1.0 / self.control_freq
            if self.freq_estimator.last_time:
                if time.time() - self.freq_estimator.last_time < interval:
                    # Wait until the next control cycle
                    time.sleep(interval - (time.time() - self.freq_estimator.last_time))

            # === Frequency computation ===
            self.freq_estimator.update()

    def stop(self):
        self.stop_event.set()

    def _is_between_phase(self, phase, start, end):
        """Returns True if phase is within [start, end] accounting for wrap-around"""
        if start < end:
            return start <= phase <= end
        else:
            return phase >= start or phase <= end
