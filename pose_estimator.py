import threading
import time
from collections import deque
import pandas as pd
from timing_utils import FrequencyEstimator
import math
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Angle and Direction Convention (used throughout PoseEstimator)
#
# - The Center of Pressure (CoP) is computed using weighted positions from
#   the pressure sensor array.
#
# - The angle is calculated as:
#       angle = atan2(cop_y, cop_x)
#   and returned in degrees.
#
# - This angle follows standard polar coordinates:
#       0°   = Positive X direction (East)
#       90°  = Positive Y direction (North)
#       180° = Negative X direction (West)
#       270° = Negative Y direction (South)
#
# - The value wraps in [0°, 360°), increasing counter-clockwise.
#
# - This convention ensures compatibility with how azimuth is used elsewhere
#   in the system (e.g., actuator targeting, control visualization).
#
# - Angle estimation includes optional smoothing of CoP values via exponential
#   filtering, to reduce jitter from sensor noise.
# ─────────────────────────────────────────────────────────────────────────────


class PoseEstimator(threading.Thread):
    def __init__(self, source, history_size=1000):
        super().__init__(daemon=True, name="PoseEstimator")
        self.history = deque(maxlen=history_size)
        self.reversal_angles = deque(maxlen=10)
        self.last_velocity = 0.0
        self.last_reversal = None
        self.last_reversal_time = None

        # Filters
        self.filtered_cop_x = None
        self.filtered_cop_y = None
        self.last_update_time = None
        self.position_filter_tau = 0.2  # seconds

        self.filtered_velocity = 0.0
        self.velocity_filter_tau = 0.2  # Larger value for more smoothing
        self.velocity_filter_last_time = None

        self.filtered_acceleration = 0.0
        self.acceleration_filter_tau = 0.5  # Larger value for more smoothing
        self.acceleration_filter_last_time = None

        self.state = {}
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ready_flag = threading.Event()
        self.finished_flag = threading.Event()

        # Reversal tracking parameters
        self.reversal_times = deque(maxlen=20)
        self.DEADBAND_THRESHOLD = 0.0

        # Circle fitting parameters
        self.arc_length_min = 150  # look back for this many mm
        self.arc_history_min_points = 5  # minimum number of points to fit an arc
        self.arc_history_time_sec_max = (
            3.0  # look further back this many seconds if distance is not enough
        )
        self.arc_portion_deg_min = 5.0  # minimum angle portion to consider a valid arc
        self.arc_filter_tau = (
            3.0  # Larger value for more smoothing # 4 seconds seems reasonable
        )
        self.arc_filter_last_time = None
        self.expected_radius = 115.0  # Expected radius in mm for the arc
        self.expected_radius_clamp = [60,180]
        self.arc_filtered_center_x = 0  # None # Initialize to 0 for warm start
        self.arc_filtered_center_y = 0  # None # Initialize to 0 for warm start
        self.arc_filtered_radius = self.expected_radius  # Initialize to expected radius
        self.arc_filtered_center_clamp = [-250, 250]

        self.stationary_time_window = 4.0
        self.stationary_velocity_threshold = 4.0

        self.freq_estimator = FrequencyEstimator(alpha=0.2)

        # Handle source
        if isinstance(source, str):
            self.mode = "csv"
            self.df = pd.read_csv(source)
            self.df_iter = self.df.iterrows()
        else:
            self.mode = "stream"
            self.data_stream = source

    def stop(self):
        self.stop_event.set()

    def is_ready(self):
        return self.ready_flag.is_set()  # <--- NEW

    def get_latest_state(self):
        with self.lock:
            return self.state.copy()

    def run(self):
        if self.mode == "csv":
            self.start_time = self.df["Time (s)"].iloc[0]
            self.start_wall_time = time.time()

            for _, row in self.df_iter:
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

        angle = self._estimate_angle(row["cop_x"], row["cop_y"])
        velocity = self._estimate_velocity()
        acceleration = self._estimate_acceleration()
        c_x, c_y, c_r = self.estimate_arc_center()
        self._check_velocity_reversal()

        if self.last_velocity * velocity < 0:
            self.last_reversal = angle
            self.reversal_angles.append(angle)

        self.last_velocity = velocity

        phase = self.get_phase()
        direction = self.get_direction()
        motion_state = self._classify_motion_state()
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
            "motion_state": motion_state,
            "arc_center_x": c_x,
            "arc_center_y": c_y,
            "arc_radius": c_r,
        }

        # === Frequency computation ===
        self.freq_estimator.update()

        # Update shared state and history
        with self.lock:
            self.state.update(state_now)
            self.history.append(state_now)

        self.ready_flag.set()

    def wrapped_angle_diff(self, a1, a0):
        """Compute smallest angular difference (in degrees) considering wraparound at ±180°."""
        diff = (a1 - a0 + 180) % 360 - 180
        return diff

    def _estimate_angle(self, raw_cop_x, raw_cop_y):
        now = time.time()

        if self.filtered_cop_x is None:
            # Initialize
            self.filtered_cop_x = raw_cop_x
            self.filtered_cop_y = raw_cop_y
            self.last_update_time = now
            return math.degrees(math.atan2(raw_cop_y, raw_cop_x))

        dt = now - self.last_update_time
        self.last_update_time = now

        alpha = 1.0 - math.exp(-dt / self.position_filter_tau)

        self.filtered_cop_x = (1 - alpha) * self.filtered_cop_x + alpha * raw_cop_x
        self.filtered_cop_y = (1 - alpha) * self.filtered_cop_y + alpha * raw_cop_y

        angle_rad = math.atan2(self.filtered_cop_y, self.filtered_cop_x)
        return math.degrees(angle_rad)

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

    def _classify_motion_state(self):
        now = time.time()

        # --- 1. Stationary/Vibration ---
        if len(self.history) >= 2:
            recent_window = [
                s
                for s in self.history
                if now - s["timestamp"] < self.stationary_time_window
            ]
            # if recent_window and all(
            #     abs(s["velocity"]) < self.stationary_velocity_threshold
            #     for s in recent_window
            # ):
            if recent_window :
                total = sum(abs(s["velocity"]) for s in recent_window)
                average = total / len(recent_window)
                if average < self.stationary_velocity_threshold:
                    return 1  # Stationary

        # --- 2. Oscillation vs Full Rotation ---
        # Find index of last reversal
        if self.last_reversal_time is None:
            return 2  # Oscillation (no reversal yet)

        accumulated_angle = 0.0
        prev_angle = None
        found = False

        # Go backwards through history until last reversal
        for s in reversed(self.history):
            t = s["timestamp"]
            angle = s["angle"]

            if t < self.last_reversal_time:
                break

            if prev_angle is not None:
                delta = self.wrapped_angle_diff(angle, prev_angle)
                accumulated_angle += abs(delta)
            prev_angle = angle

            if accumulated_angle >= 360:
                return 3  # Full Rotation

        return 2  # Oscillation

    def get_phase(self):
        """
        Returns:
            phase (float): phase angle in degrees from 0 to 360.
                0–180: CW direction
                180–360: CCW direction
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
        normalized_phase = min(
            1.0, max(0.0, t_since_last / half_period)
        )  # clamp to [0,1]

        # Use velocity sign to infer direction
        latest_velocity = self.state.get("velocity", 0.0)
        if latest_velocity >= 0:
            return normalized_phase * 180.0  # CW half
        else:
            return 180.0 + normalized_phase * 180.0  # CCW half

    def get_direction(self):
        """
        Returns:
            True if moving in positive direction, False if negative.
        """
        return self.state.get("velocity", 0.0) > 0

    def _fit_circle_numpy(self, points):
        """
        Fit circle using linear least squares.
        Returns: (center_x, center_y, radius)
        """
        x = np.array([p[0] for p in points])
        y = np.array([p[1] for p in points])
        A = np.c_[2 * x, 2 * y, np.ones(len(points))]
        b = x**2 + y**2

        try:
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx, cy = sol[0], sol[1]
            r = np.sqrt(sol[2] + cx**2 + cy**2)
            return cx, cy, r
        except np.linalg.LinAlgError:
            return None, None, None

    # def estimate_arc_center(self):
    #     now = time.time()
    #     points = []
    #     total_arc_length = 0.0
    #     last_point = None

    #     for state in reversed(self.history):
    #         age = now - state["timestamp"]
    #         if age > self.arc_history_time_sec_max:
    #             break

    #         x = state.get("cop_x")
    #         y = state.get("cop_y")
    #         if x is None or y is None:
    #             continue

    #         current_point = (x, y)
    #         if last_point:
    #             dist = math.hypot(x - last_point[0], y - last_point[1])
    #             total_arc_length += dist

    #         points.append(current_point)
    #         last_point = current_point

    #         if (
    #             total_arc_length >= self.arc_length_min
    #             and len(points) >= self.arc_history_min_points
    #         ):
    #             break  # Stop once we accumulate enough arc length

    #     # if len(points) < 10:
    #     #     return  # Too few points
    #     if total_arc_length < self.arc_length_min:
    #         return

    #     points = list(reversed(points))  # chronological order

    #     # === Fit circle ===
    #     cx, cy, r = self._fit_circle_numpy(points)
    #     if cx is None or cy is None or r is None or r == 0:
    #         return

    #     # === Validate angle portion ===
    #     # angles = [math.atan2(p[1] - cy, p[0] - cx) for p in points]
    #     # min_angle = min(angles)
    #     # max_angle = max(angles)
    #     # arc_angle_deg = math.degrees(
    #     #     (max_angle - min_angle + math.pi * 2) % (math.pi * 2)
    #     # )

    #     # if arc_angle_deg < self.arc_portion_deg_min:
    #     #     return  # arc too short to be reliable

    #     # === Exponential smoothing ===
    #     if self.arc_filter_last_time is None:
    #         alpha = 1.0
    #     else:
    #         dt = now - self.arc_filter_last_time
    #         alpha = 1 - math.exp(-dt / self.arc_filter_tau)

    #     self.arc_filter_last_time = now

    #     def smooth(old, new):
    #         return alpha * new + (1 - alpha) * old if old is not None else new

    #     self.arc_filtered_center_x = smooth(self.arc_filtered_center_x, cx)
    #     self.arc_filtered_center_y = smooth(self.arc_filtered_center_y, cy)
    #     self.arc_filtered_radius = smooth(self.arc_filtered_radius, r)
    #     print(
    #         f"Arc Center: ({self.arc_filtered_center_x:.2f}, {self.arc_filtered_center_y:.2f}), Radius: {self.arc_filtered_radius:.2f} m"
    #     )

    def subsample_points(self, points, max_points=25):
        """
        Sub-samples a list of (x, y) points to ensure the length does not exceed max_points.
        Returns a new list of evenly spaced points.
        """
        n = len(points)
        if n <= max_points:
            return points  # no subsampling needed

        step = n / max_points
        indices = [int(i * step) for i in range(max_points)]
        return [points[i] for i in indices]

    def estimate_arc_center(self):
        """
        Fit a circle to the current motion arc assuming a soft constraint on radius.
        Updates:
            - self.arc_filtered_center_x
            - self.arc_filtered_center_y
            - self.arc_filtered_radius
        """
        now = time.time()

        points = []
        total_arc_length = 0.0
        last_point = None

        for state in reversed(self.history):
            age = now - state["timestamp"]
            if age > self.arc_history_time_sec_max:
                break

            x = state.get("cop_x")
            y = state.get("cop_y")
            if x is None or y is None:
                continue

            current_point = (x, y)
            if last_point:
                dist = math.hypot(x - last_point[0], y - last_point[1])
                total_arc_length += dist

            points.append(current_point)
            last_point = current_point

            if (
                total_arc_length >= self.arc_length_min
                and len(points) >= self.arc_history_min_points
            ):
                break  # Stop once we accumulate enough arc length

        # if len(points) < 10:
        #     return  # Too few points
        if total_arc_length < self.arc_length_min:
            return (
                self.arc_filtered_center_x,
                self.arc_filtered_center_y,
                self.arc_filtered_radius,
            )

        points = self.subsample_points(points, max_points=25)
        # points.reverse()  # To preserve time order

        # 2. Start from previous or average
        xs, ys = zip(*points)
        cx = (
            self.arc_filtered_center_x
            if self.arc_filtered_center_x is not None
            else sum(xs) / len(xs)
        )
        cy = (
            self.arc_filtered_center_y
            if self.arc_filtered_center_y is not None
            else sum(ys) / len(ys)
        )
        r = (
            self.arc_filtered_radius
            if self.arc_filtered_radius is not None
            else self.expected_radius
        )

        # 3. Gradient Descent
        learning_rate = 0.001
        radius_weight = 0.1
        iterations = 5

        for _ in range(iterations):
            grad_cx = grad_cy = grad_r = 0.0
            total_weight = 0

            for x, y in points:
                dx = cx - x
                dy = cy - y
                dist = math.hypot(dx, dy)
                if dist < 1e-5:
                    continue  # skip degenerate points

                error = dist - r
                grad_cx += (error * dx) / dist
                grad_cy += (error * dy) / dist
                grad_r += -error
                total_weight += 1

            if total_weight == 0:
                return (
                    self.arc_filtered_center_x,
                    self.arc_filtered_center_y,
                    self.arc_filtered_radius,
                )

            # Normalize gradients
            grad_cx /= total_weight
            grad_cy /= total_weight
            grad_r /= total_weight

            # Soft radius penalty
            r_error = r - self.expected_radius
            grad_r += radius_weight * r_error

            # Update
            cx -= learning_rate * grad_cx
            cy -= learning_rate * grad_cy
            r -= learning_rate * grad_r

        # 4. Smooth result
        alpha = self.arc_filter_tau
        if self.arc_filtered_center_x is None:
            self.arc_filtered_center_x = cx
            self.arc_filtered_center_y = cy
            self.arc_filtered_radius = r
        else:
            self.arc_filtered_center_x = (
                alpha * cx + (1 - alpha) * self.arc_filtered_center_x
            )
            self.arc_filtered_center_y = (
                alpha * cy + (1 - alpha) * self.arc_filtered_center_y
            )
            self.arc_filtered_radius = (
                alpha * r + (1 - alpha) * self.arc_filtered_radius
            )

        # 5. Clamp values to sainity range
        if self.arc_filtered_radius < self.expected_radius_clamp[0]:
            self.arc_filtered_radius = self.expected_radius_clamp[0]
        if self.arc_filtered_radius > self.expected_radius_clamp[1]:
            self.arc_filtered_radius = self.expected_radius_clamp[1]

        if self.arc_filtered_center_x < self.arc_filtered_center_clamp[0]:
            self.arc_filtered_center_x = self.arc_filtered_center_clamp[0]
        if self.arc_filtered_center_x > self.arc_filtered_center_clamp[1]:
            self.arc_filtered_center_x = self.arc_filtered_center_clamp[1]

        if self.arc_filtered_center_y < self.arc_filtered_center_clamp[0]:
            self.arc_filtered_center_y = self.arc_filtered_center_clamp[0]
        if self.arc_filtered_center_y > self.arc_filtered_center_clamp[1]:
            self.arc_filtered_center_y = self.arc_filtered_center_clamp[1]

        

        print(
            f"[CircleFit] Center: ({self.arc_filtered_center_x:.2f}, {self.arc_filtered_center_y:.2f}), "
            f"Radius: {self.arc_filtered_radius:.2f} mm, "
            f"[ArcFit] Using {len(points)} points for circle fit."
        )
        return (
            self.arc_filtered_center_x,
            self.arc_filtered_center_y,
            self.arc_filtered_radius,
        )
