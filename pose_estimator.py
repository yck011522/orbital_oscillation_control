import threading
import time
from collections import deque
import pandas as pd


class PoseEstimator:
    def __init__(self, source, history_size=1000):
        self.history = deque(maxlen=history_size)
        self.reversal_angles = deque(maxlen=10)
        self.last_velocity = 0.0
        self.last_reversal = None

        # Filters
        self.filtered_velocity = 0.0
        self.velocity_filter_tau = 0.05
        self.velocity_filter_last_time = None

        self.filtered_acceleration = 0.0
        self.acceleration_filter_tau = 0.2
        self.acceleration_filter_last_time = None

        self.state = {}
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ready_flag = threading.Event()
        self.finished_flag = threading.Event()

        self.reversal_times = deque(maxlen=20)
        self.DEADBAND_THRESHOLD = 0.0

        # Handle source
        if isinstance(source, str):
            self.mode = "csv"
            self.df = pd.read_csv(source)
            self.df_iter = self.df.iterrows()
        else:
            self.mode = "stream"
            self.data_stream = source

    def start(self):
        if self.mode == "csv":
            self.start_time = self.df["Time (s)"].iloc[0]
            self.start_wall_time = time.time()
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.stop_event.set()

    def is_ready(self):
        return self.ready_flag.is_set()  # <--- NEW

    def get_latest_state(self):
        with self.lock:
            return self.state.copy()

    def _run(self):
        if self.mode == "csv":
            for idx, row in self.df_iter:
                if self.stop_event.is_set():
                    break

                sim_time = row["Time (s)"]
                target_wall_time = self.start_wall_time + (sim_time - self.start_time)
                while time.time() < target_wall_time:
                    time.sleep(0.001)

                self._process_row(
                    {
                        "angle": row["Angle (deg)"],
                        "cop_x": row["COP_X (m)"],
                        "cop_y": row["COP_Y (m)"],
                        "total_weight": row["TotalWeight (kg)"],
                    }
                )

            self.finished_flag.set()

        elif self.mode == "stream":
            for _, row in self.data_stream:
                if self.stop_event.is_set():
                    break

                self._process_row(row)

    def _process_row(self, row):
        now = time.time()
        angle = row["angle"]

        velocity = self._estimate_velocity()
        acceleration = self._estimate_acceleration()
        self._check_velocity_reversal()

        if self.last_velocity * velocity < 0:
            self.last_reversal = angle
            self.reversal_angles.append(angle)

        self.last_velocity = velocity

        phase = self.get_phase()
        direction = self.get_direction()

        state_now = {
            "timestamp": now,
            "angle": angle,
            "velocity": velocity,
            "acceleration": acceleration,
            "last_reversal_angle": self.last_reversal,
            "total_weight": row["total_weight"],
            "cop_x": row["cop_x"],
            "cop_y": row["cop_y"],
            "phase": phase,
            "direction_positive": direction,
        }

        with self.lock:
            self.state.update(state_now)
            self.history.append(state_now)

        self.ready_flag.set()

    def wrapped_angle_diff(self, a1, a0):
        """Compute smallest angular difference (in degrees) considering wraparound at ±180°."""
        diff = (a1 - a0 + 180) % 360 - 180
        return diff

    def _estimate_velocity(self, window=5):
        if len(self.history) < window:
            return 0.0

        s0 = self.history[-window]
        s1 = self.history[-1]
        t0, a0 = s0["timestamp"], s0["angle"]
        t1, a1 = s1["timestamp"], s1["angle"]

        # Use wrapped angle difference
        angle_delta = self.wrapped_angle_diff(a1, a0)
        raw_velocity = angle_delta / (t1 - t0 + 1e-6)

        now = time.time()
        dt_filter = (
            now - self.velocity_filter_last_time
            if self.velocity_filter_last_time
            else 1e-3
        )
        self.velocity_filter_last_time = now
        alpha = dt_filter / (self.velocity_filter_tau + dt_filter)
        self.filtered_velocity = (
            alpha * raw_velocity + (1 - alpha) * self.filtered_velocity
        )

        return self.filtered_velocity

    def _estimate_acceleration(self, window=5):
        if len(self.history) < window + 2:
            return 0.0

        # Get earlier and later windows for velocity estimation
        s0 = self.history[-(window + 2)]
        s1 = self.history[-window]
        s2 = self.history[-window]
        s3 = self.history[-1]

        t0, a0 = s0["timestamp"], s0["angle"]
        t1, a1 = s1["timestamp"], s1["angle"]
        t2, a2 = s2["timestamp"], s2["angle"]
        t3, a3 = s3["timestamp"], s3["angle"]

        # Use wrapped angle diffs for velocity estimates
        v0 = self.wrapped_angle_diff(a1, a0) / (t1 - t0 + 1e-6)
        v1 = self.wrapped_angle_diff(a3, a2) / (t3 - t2 + 1e-6)
        dt = t3 - t0

        raw_acceleration = (v1 - v0) / (dt + 1e-6)

        # Low-pass filter
        now = time.time()
        dt_filter = (
            now - self.acceleration_filter_last_time
            if self.acceleration_filter_last_time
            else 1e-3
        )
        self.acceleration_filter_last_time = now
        alpha = dt_filter / (self.acceleration_filter_tau + dt_filter)
        self.filtered_acceleration = (
            alpha * raw_acceleration + (1 - alpha) * self.filtered_acceleration
        )

        return self.filtered_acceleration

    def is_finished(self):
        return self.finished_flag.is_set()

    def _check_velocity_reversal(self):
        if len(self.history) < 2:
            return

        s_prev = self.history[-2]
        s_curr = self.history[-1]

        v_prev = s_prev.get("velocity", 0.0)
        v_curr = s_curr.get("velocity", 0.0)

        MIN_INTERVAL = 0.5  # seconds, adjust as needed

        if v_prev * v_curr < 0:
            t_prev = s_prev["timestamp"]
            t_curr = s_curr["timestamp"]

            alpha = abs(v_prev) / (abs(v_prev) + abs(v_curr))
            t_reversal = t_prev + alpha * (t_curr - t_prev)

            if (
                self.reversal_times
                and (t_reversal - self.reversal_times[-1][0]) < MIN_INTERVAL
            ):
                # Too close to last reversal, skip
                return

            angle_prev = s_prev.get("angle", 0.0)
            angle_curr = s_curr.get("angle", 0.0)
            angle_reversal = angle_prev + alpha * (angle_curr - angle_prev)

            self.reversal_times.append((t_reversal, angle_reversal))
            self.last_reversal_time = t_reversal
            self.last_reversal_angle = angle_reversal
            v_prev = s_prev.get("velocity", 0.0)

    def get_phase(self):
        """
        Returns:
            phase (float): between 0.0 and 1.0
        """
        if len(self.reversal_times) < 2:
            return 0.0

        now = time.time()
        t_prev, _ = self.reversal_times[-2]
        t_curr, _ = self.reversal_times[-1]

        half_period = t_curr - t_prev
        if half_period < 1e-6:
            return 0.0

        t_since_last = now - t_curr
        phase = t_since_last / half_period

        return min(1.0, max(0.0, phase))  # clamp to [0,1]

    def get_direction(self):
        """
        Returns:
            True if moving in positive direction, False if negative.
        """
        return self.state.get("velocity", 0.0) > 0
