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

        self.max_tilt = 0.5
        self.acceleration_rate = 1.0  # deg/s²
        self.deceleration_rate = 1.0  # deg/s²

        self.current_tilt = 0.0
        self.current_velocity = 0.0

        # Controller state
        self.last_phase = None
        self.triggered = False

        # Frequency tracking
        self.freq_estimator = FrequencyEstimator(alpha=0.9)

    def update(self, current_phase, dt):
        """
        Update the tilt control logic based on current phase.
        `current_phase` should be in the range [0.0, 2.0].
        `dt` is the time step in seconds.
        """
        p = current_phase
        p_start = self.phase_start
        p_end = self.phase_end

        # Normalize phase so it wraps around 2.0
        def in_range(x, start, end):
            if start <= end:
                return start <= x <= end
            else:
                return x >= start or x <= end

        if in_range(p, p_start, p_end):
            # Within actuation range
            if self.current_tilt < self.max_tilt:
                self.current_velocity += self.acceleration_rate * dt
            else:
                self.current_velocity = 0.0  # Hold
        else:
            # Outside actuation range
            if self.current_tilt > 0.0:
                self.current_velocity -= self.deceleration_rate * dt
            else:
                self.current_velocity = 0.0  # Return to zero tilt

        # Integrate velocity to update tilt
        self.current_tilt += self.current_velocity * dt

        # Clamp tilt and velocity
        if self.current_tilt > self.max_tilt:
            self.current_tilt = self.max_tilt
            self.current_velocity = 0.0
        elif self.current_tilt < 0.0:
            self.current_tilt = 0.0
            self.current_velocity = 0.0

    def run(self):

        while not self.pose_estimator.is_ready():
            time.sleep(0.01)

        while not self.pose_estimator.is_finished() and not self.stop_event.is_set():

            state = self.pose_estimator.get_latest_state()
            phase = self.pose_estimator.get_phase()
            direction = self.pose_estimator.get_direction()
            dt = (
                time.time() - self.freq_estimator.last_time
                if self.freq_estimator.last_time
                else 0.01
            )
            self.update(phase, dt)

            print(
                f"Timestamp: {state['timestamp']:.2f}, COP: ({state['cop_x']:.2f}, {state['cop_y']:.2f}), "
                + f"θ: {state['angle']:.2f}, v: {state['velocity']:.2f}, a: {state['acceleration']:.2f}"
                + f", Phase: {phase:.2f}, Direction: {'Forward' if direction else 'Backward'}"
                + f", Motion State: {state['motion_state']}"
                + f", Current Tilt: {self.current_tilt:.2f}, Velocity: {self.current_velocity:.2f}"
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
                if dt < interval:
                    # Wait until the next control cycle
                    time.sleep(interval - dt)

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
