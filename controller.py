import threading
import time


class Controller(threading.Thread):
    def __init__(self, pose_estimator, control_freq=25):
        super().__init__(daemon=True)
        self.pose_estimator = pose_estimator
        self.control_freq = control_freq
        self.stop_event = threading.Event()

        # Controller state
        self.last_phase = None
        self.triggered = False

    def run(self):
        interval = 1.0 / self.control_freq

        while not self.pose_estimator.is_ready():
            time.sleep(0.01)

        while not self.pose_estimator.is_finished() and not self.stop_event.is_set():
            state = self.pose_estimator.get_latest_state()
            phase = self.pose_estimator.get_phase()
            direction = self.pose_estimator.get_direction()

            print(
                f"Timestamp: {state['timestamp']:.2f}, COP: ({state['cop_x']:.2f}, {state['cop_y']:.2f}), "
                + f"θ: {state['angle']:.2f}, v: {state['velocity']:.2f}, a: {state['acceleration']:.2f}"
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
            time.sleep(interval)

    def stop(self):
        self.stop_event.set()
