import threading
import time
from motor_control import send_absolute_position_mm, open_serial, home_all_motors
from actuator_utils import actuator_length, polar_to_tilt_components

from timing_utils import FrequencyEstimator
from pose_estimator import PoseEstimator
import math

ABS_MAX_TILT = 0.9

class Controller(threading.Thread):
    def __init__(
        self, pose_estimator: PoseEstimator, control_freq=100, live_output=True
    ):
        super().__init__(daemon=True, name="Controller")
        self.pose_estimator = pose_estimator
        self.live_output = live_output

        self.control_freq = control_freq
        self.stop_event = threading.Event()

        # FSM states
        self.STATE_WAIT_WHILE_STATIONARY = 0
        self.STATE_DELAY_BEFORE_PUMP = 1
        self.STATE_PUMP = 2
        self.STATE_DECAY = 3

        self.state = self.STATE_WAIT_WHILE_STATIONARY
        self.control_method = ""
        self.debug_force_state = None  # set this to 2 (PUMP) to force debug mode

        # Timing and parameters
        self.delay_timer_start = None
        self.full_rotation_start_time = time.time()
        self.full_rotation_duration = 12.0
        self.wait_time_after_stationary = 40.0
        self.stationary_begin_time = 0
        self.delay_duration = 2.0

        # Oscillation control mode parameters
        self.oscillation_tilt = 0.0
        self.oscillation_init_direction = True  # True for positive direction
        self.oscillation_pause_start = time.time()

        self.starting_acceleration_rate = 1.02
        self.starting_max_tilt = 1.08
        self.starting_pause_time = 0.53       

        # Phase control parameters
        self.phase_start = 95
        self.phase_end = 260
        self.pump_max_tilt_initial = 0.70  # degrees
        self.pump_max_tilt_regular = 0.24  # degrees
        self.pump_acceleration_rate_initial = 0.53  # deg/sec²
        self.pump_acceleration_rate_regular = 0.22  # deg/sec²
        self.pump_initial_rate_duration = 2.0 # seconds
        self.pump_max_tilt = self.pump_max_tilt_initial
        self.pump_acceleration_rate = self.pump_acceleration_rate_initial
        self.pump_oscillation_start_time = time.time()
        self.lead_angle_deg = 90  # lead angle for azimuth

        # Full rotation control mode parameters
        self.full_rotation_tilt = 0.11  # degrees (constant tilt to maintain)
        self.full_rotation_lead_angle = 90.0  # degrees ahead of current object angle

        # Decay Finish parameters
        self.decay_finish_velocity_threshold = 20.0 # Decay finished when velocity below this (deg/s) for over x seconds
        self.decay_finish_duration = 6.0  # seconds

        # Center restoring vector parameters
        self.center_restoring_gain = 0.0195  # Gain for restoring vector towards center

        # Control output parameters
        self.target_tilt = 0.0  # Current target tilt angle in degrees
        self.target_azimuth = 0.0  # Current target azimuth angle in degrees

        # Rate limiting (Cartesian space)
        self.max_vector_rate_per_sec = 0.1  # Maximum rate of change of tilt vector per second
        self.limited_tilt_vector = (0.0, 0.0)  # Current rate-limited tilt vector

        # Motor control parameters
        self.prev_positions = {1: None, 2: None, 3: None, 4: None}
        self.actuator_value_offset = 0.2
        self.default_speed_rpm = 100
        self.speed_adjustment = 1.0
        self.last_motor_time = time.time()

        # Frequency tracking
        self.last_control_time = time.time()
        self.delta_time = 0.0
        self.freq_estimator = FrequencyEstimator(alpha=0.2)

        # Serial connection for motor control
        if self.live_output:
            self.ser = open_serial()
            time.sleep(5.0)
            homing_result = home_all_motors(self.ser, settle_position_mm=28.1)
            if not homing_result:
                print("⚠️ Homing failed. Exiting controller.")
                exit()
        else:
            print("Motors homing skipped in non-live mode.")

    def get_state_text(self):
        state_names = {
            self.STATE_WAIT_WHILE_STATIONARY: "WAIT_WHILE_STATIONARY",
            self.STATE_DELAY_BEFORE_PUMP: "DELAY_BEFORE_PUMP",
            self.STATE_PUMP: "PUMP",
            self.STATE_DECAY: "DECAY",
        }
        return state_names.get(self.state, "UNKNOWN_STATE")

    def run(self):

        while not self.pose_estimator.is_ready():
            time.sleep(0.01)

        while not self.pose_estimator.is_finished() and not self.stop_event.is_set():
            self.delta_time = time.time() - self.last_control_time

            # Sleep till this control cycle begins
            interval = 1.0 / self.control_freq
            if self.delta_time < interval:
                time.sleep(interval - self.delta_time)
                self.delta_time = time.time() - self.last_control_time
            self.last_control_time = time.time()

            object_state = self.pose_estimator.get_latest_state()

            # Check debug override
            if self.debug_force_state is not None:
                self.state = self.debug_force_state
            else:
                self.state = self.state

            # FSM Logic / Switch between control states
            control_vector = (0.0, 0.0)
            if self.state == self.STATE_WAIT_WHILE_STATIONARY:
                self.control_method = "wait till stationary"
                if time.time() - self.stationary_begin_time > self.wait_time_after_stationary:
                    self.delay_timer_start = time.time()
                    self.state = self.STATE_PUMP

            # elif self.state == self.STATE_DELAY_BEFORE_PUMP:
            #     self.control_method = "before pump"
            #     if time.time() - self.delay_timer_start > self.delay_duration:
            #         self.state = self.STATE_PUMP

            elif self.state == self.STATE_PUMP:
                # Pump control logic
                if object_state["motion_state"] == 1:  # stationary
                    self.control_method = "start_from_stationary"
                    control_vector = self.control_start_from_stationary(object_state)
                    self.pump_max_tilt = self.pump_max_tilt_initial
                    self.pump_acceleration_rate = self.pump_acceleration_rate_initial
                    self.pump_oscillation_start_time = time.time()
                elif object_state["motion_state"] == 2:  # oscillation
                    self.control_method = "pump_oscillation"
                    control_vector = self.control_pump_oscillation(object_state)
                elif object_state["motion_state"] == 3:  # full rotation
                    self.control_method = "maintain_rotation"
                    control_vector = self.control_maintain_rotation(object_state)
                    self.pump_max_tilt = self.pump_max_tilt_initial
                    self.pump_acceleration_rate = self.pump_acceleration_rate_initial
                    self.pump_oscillation_start_time = time.time()
                else:
                    print(f"Unknown motion state: {object_state['motion_state']}")

                # Logic to transition to decay state
                if object_state["motion_state"] == 3:  # Full rotation reached
                    if time.time() - self.full_rotation_start_time > self.full_rotation_duration:
                        self.state = self.STATE_DECAY
                else:
                    self.full_rotation_start_time = time.time()
                
                # print(control_vector)

            elif self.state == self.STATE_DECAY:
                self.control_method = "decay"
                # Optional: graceful decay logic
                if object_state["motion_state"] == 2: # slow oscillation
                    if self.pose_estimator.get_max_velocity_history(self.decay_finish_duration) < self.decay_finish_velocity_threshold:
                        print("Decay finished - object motion below threshold.")
                        self.stationary_begin_time = time.time()
                        self.state = self.STATE_PUMP

                if object_state["motion_state"] == 1:  # object have decayed to a stop
                    self.stationary_begin_time = time.time()
                    self.state = self.STATE_WAIT_WHILE_STATIONARY

            # Compute center restoring vector
            center_restoring_vector = self.control_center_restoring_vector(object_state)

            # Send target to actuator here
            total_target_vector = (
                control_vector[0] + center_restoring_vector[0],
                control_vector[1] + center_restoring_vector[1],
            )


            # === Rate limit the tilt vector (Cartesian space) ===
            # This prevents jerky transitions between control policies by constraining
            # the maximum rate of change in the XY vector space
            self.limited_tilt_vector = self.rate_limit_tilt_vector(
                total_target_vector, self.limited_tilt_vector, self.delta_time
            )
                       
            self.target_tilt, self.target_azimuth = self.tilt_vector_to_tilt_azimuth(
                self.limited_tilt_vector
            )

            self.send_target_to_motor(self.target_tilt, self.target_azimuth)

            self.freq_estimator.update()

        if self.live_output:
            print("Controller finished. Sending final position to motors.")
            # Send final position to motors before exiting
            send_absolute_position_mm(self.ser, 21.5, 0, speed_rpm=200)
            self.ser.close()

    def stop(self):
        self.stop_event.set()

    # === Control Functions ===
    def control_start_from_stationary(self, state):
        obj_angle_deg = state["angle"]
        azimuth_deg = obj_angle_deg - self.lead_angle_deg
        # Normalize azimuth to [-180, 180] or [0, 360] if needed
        azimuth_deg = (azimuth_deg + 360) % 360

        if getattr(self, "oscillation_init_direction", None) is None:
            self.oscillation_init_direction = True
        if getattr(self, "oscillation_tilt", None) is None:
            self.oscillation_tilt = 0.0

        # Create a simple oscillation effect back and forth
        
        if self.oscillation_init_direction:
            if (time.time() - self.oscillation_pause_start) > self.starting_pause_time:
                self.oscillation_tilt += self.starting_acceleration_rate * self.delta_time
            if self.oscillation_tilt >= self.starting_max_tilt:
                self.oscillation_init_direction = False
                self.oscillation_pause_start = time.time()
                self.oscillation_tilt = self.starting_max_tilt
        else:
            if (time.time() - self.oscillation_pause_start) > self.starting_pause_time:
                self.oscillation_tilt -= self.starting_acceleration_rate * self.delta_time
            if self.oscillation_tilt <= 0.0:
                self.oscillation_init_direction = True
                self.oscillation_pause_start = time.time()
                self.oscillation_tilt = 0.0

        return self.tilt_azimuth_to_vector(self.oscillation_tilt, azimuth_deg)

    def control_pump_oscillation(self, state):
        """
        Calculates target tilt and azimuth based on oscillation phase.
        This function is called during the PUMP controller state.
        """

        phase_deg = state["phase"]
        phase_deg = phase_deg % 360  # Normalize to [0, 360]
        obj_angle_deg = state["angle"]

        if getattr(self, "oscillation_tilt", None) is None:
            self.oscillation_tilt = 0.0

        # === TILT CONTROL ===
        in_active_range = self.phase_start <= phase_deg <= self.phase_end

        # Gradual increase acc_rate and max_tilt
        if (time.time() - self.pump_oscillation_start_time) < self.pump_initial_rate_duration:
            self.pump_acceleration_rate += (self.delta_time / self.pump_initial_rate_duration) * (self.pump_acceleration_rate_regular - self.pump_acceleration_rate_initial)
            self.pump_max_tilt += (self.delta_time / self.pump_initial_rate_duration) * (self.pump_max_tilt_regular - self.pump_max_tilt_initial)
        else:
            self.pump_acceleration_rate  = self.pump_acceleration_rate_regular
            self.pump_max_tilt = self.pump_max_tilt_regular

        # print("[pump_acceleration_rate]", self.pump_acceleration_rate, "[pump_max_tilt]", self.pump_max_tilt)



        if in_active_range:
            # Accelerate upward
            # print("Accelerate upward")
            self.oscillation_tilt += self.pump_acceleration_rate * self.delta_time
            self.oscillation_tilt = min(self.oscillation_tilt, self.pump_max_tilt)
        else:
            # Decelerate downward
            # print("Decelerate downward")
            self.oscillation_tilt -= self.pump_acceleration_rate * self.delta_time
            self.oscillation_tilt = max(self.oscillation_tilt, 0.0)

        # === AZIMUTH CONTROL ===
        # Lead angle applied only during deceleration half (e.g., CW→CCW)
        # if 90 <= phase_deg <= 270:
        #     azimuth_deg = obj_angle_deg - self.lead_angle_deg
        # else:
        #     azimuth_deg = obj_angle_deg
        azimuth_deg = obj_angle_deg - self.lead_angle_deg

        # Normalize azimuth to [-180, 180] or [0, 360] if needed
        azimuth_deg = (azimuth_deg + 360) % 360

        # === Return result ===
        return self.tilt_azimuth_to_vector(self.oscillation_tilt, azimuth_deg)

    def control_maintain_rotation(self, state):
        """
        Maintains a constant tilt and points azimuth slightly ahead of the object
        to sustain rotation.
        """

        obj_angle_deg = state["angle"]
        direction_is_forward = state["direction_positive"]
          # True for CW

        # === TILT ===
        tilt_deg = (
            self.full_rotation_tilt
        )  # hold constant, need to change this to adaptive

        # === AZIMUTH ===
        if direction_is_forward:
            azimuth_deg = obj_angle_deg + self.full_rotation_lead_angle
        else:
            azimuth_deg = obj_angle_deg - self.full_rotation_lead_angle

        # Normalize azimuth to [0, 360]
        azimuth_deg = (azimuth_deg + 360) % 360

        return self.tilt_azimuth_to_vector(tilt_deg, azimuth_deg)

    def control_center_restoring_vector(self, state):
        """
        Computes the center restoring vector based on the current object center of rotation.
        This vector is used to roll the object back to the center of the table.
        """
        arc_center_x = state["arc_center_x"]
        arc_center_y = state["arc_center_y"]
        arc_radius = state["arc_radius"]
        center_azimuth_deg = self.tilt_vector_to_tilt_azimuth(
            (arc_center_x, arc_center_y)
        )[1]
        center_distance_to_table_center = math.sqrt(arc_center_x**2 + arc_center_y**2)

        # Limit the length of the restoring vector
        restoring_azimuth_deg = (
            center_azimuth_deg + 180.0
        ) % 360  # Pointing towards the center

        restoring_tilt = center_distance_to_table_center * self.center_restoring_gain

        return self.tilt_azimuth_to_vector(restoring_tilt, restoring_azimuth_deg)

    def rate_limit_tilt_vector(self, target_vector, current_vector, dt):
        """
        Rate-limit the tilt vector in Cartesian space to prevent jerky transitions.
        
        This method constrains the maximum rate of change of the tilt vector,
        allowing smooth transitions between control policies without introducing lag.
        The rate limiting happens in Cartesian space (X, Y) rather than polar space
        to preserve geometric relationships and ensure smooth motor motion.
        
        The algorithm:
        1. Calculates the desired change in tilt vector (target - current)
        2. Computes the magnitude of this change
        3. If magnitude exceeds max_vector_rate_per_sec * dt, scales the change proportionally
        4. Returns the new limited vector: current + limited_change
        
        This ensures smooth transitions when:
        - Switching between control policies (e.g., PUMP → MAINTAIN)
        - Motion state oscillates (e.g., bouncing between 1↔2)
        - Sudden changes in object state occur
        
        Args:
            target_vector (tuple): Desired (x, y) tilt vector
            current_vector (tuple): Current (x, y) tilt vector (rate-limited from previous frame)
            dt (float): Time step in seconds since last control cycle
        
        Returns:
            tuple: Rate-limited (x, y) tilt vector
        """
        target_x, target_y = target_vector
        current_x, current_y = current_vector
        
        # Calculate desired change in vector
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        
        # Magnitude of desired change
        delta_magnitude = math.hypot(delta_x, delta_y)
        
        # Maximum allowed change this frame (in vector magnitude units)
        max_delta_per_frame = self.max_vector_rate_per_sec * dt
        
        if delta_magnitude > max_delta_per_frame and delta_magnitude > 1e-6:
            # Scale down the delta to fit within rate limit
            # This maintains the direction of change while limiting magnitude
            scale_factor = max_delta_per_frame / delta_magnitude
            delta_x *= scale_factor
            delta_y *= scale_factor
            
            # Debug message when rate limiting is active
            target_tilt, target_azimuth = self.tilt_vector_to_tilt_azimuth(target_vector)
            current_tilt, current_azimuth = self.tilt_vector_to_tilt_azimuth(current_vector)
            limited_tilt, limited_azimuth = self.tilt_vector_to_tilt_azimuth((current_x + delta_x, current_y + delta_y))
            
            print(f"[RATE LIMIT] Desired delta: {delta_magnitude:.4f} rad/s → Limited to: {max_delta_per_frame:.4f} rad/s | "
                  f"Target: {target_tilt:.3f}° @ {target_azimuth:.1f}° → "
                  f"Limited: {limited_tilt:.3f}° @ {limited_azimuth:.1f}°")
        
        # Apply limited change
        limited_x = current_x + delta_x
        limited_y = current_y + delta_y
        
        return (limited_x, limited_y)

    def send_target_to_motor(self, target_tilt, target_azimuth):
        now = time.time()
        dt = now - self.last_motor_time
        self.last_motor_time = now
        # Clamp target values to within tilt vector limits for SAFETY
        target_tilt = max(-ABS_MAX_TILT, min(ABS_MAX_TILT, target_tilt))

        # Convert to radians
        tilt_rad = math.radians(target_tilt)
        azimuth_rad = math.radians(target_azimuth)
        theta_x, theta_y = polar_to_tilt_components(tilt_rad, azimuth_rad)

        L_x_plus = actuator_length(theta_x) + self.actuator_value_offset
        L_x_minus = actuator_length(-theta_x) + self.actuator_value_offset
        L_y_plus = actuator_length(theta_y) + self.actuator_value_offset
        L_y_minus = actuator_length(-theta_y) + self.actuator_value_offset

        # clamp actuator targets to within soft limits for SAFETY
        L_x_plus = max(0.1, min(L_x_plus, 49.0))
        L_x_minus = max(0.1, min(L_x_minus, 49.0))
        L_y_plus = max(0.1, min(L_y_plus, 49.0))
        L_y_minus = max(0.1, min(L_y_minus, 49.0))

        target_positions = {1: L_x_plus, 2: L_y_minus, 3: L_x_minus, 4: L_y_plus}

        estimated_rpm = {}
        for motor_id, pos in target_positions.items():
            prev_pos = self.prev_positions[motor_id]
            if prev_pos is not None and dt > 0:
                velocity_mm_s = (pos - prev_pos) / dt
                rpm = abs(velocity_mm_s * 60 / 2.0)  # pitch = 2mm
                estimated_rpm[motor_id] = int(min(rpm * self.speed_adjustment, 1000))
            else:
                estimated_rpm[motor_id] = self.default_speed_rpm

        # Send to motors
        if self.live_output:
            send_absolute_position_mm(self.ser, L_x_plus, 1, speed_rpm=estimated_rpm[1])
            send_absolute_position_mm(
                self.ser, L_y_minus, 2, speed_rpm=estimated_rpm[2]
            )
            send_absolute_position_mm(
                self.ser, L_x_minus, 3, speed_rpm=estimated_rpm[3]
            )
            send_absolute_position_mm(self.ser, L_y_plus, 4, speed_rpm=estimated_rpm[4])

        self.prev_positions = target_positions

    def tilt_azimuth_to_vector(self, tilt_deg, azimuth_deg):
        """
        Converts tilt and azimuth angles in degrees to a XY vector.
        """
        tilt_rad = math.radians(tilt_deg)
        azimuth_rad = math.radians(azimuth_deg)
        x = math.sin(tilt_rad) * math.cos(azimuth_rad)
        y = math.sin(tilt_rad) * math.sin(azimuth_rad)
        return (x, y)

    def tilt_vector_to_tilt_azimuth(self, vector):
        """
        Converts a XY vector to tilt and azimuth angles in degrees.
        An inverse of tilt_azimuth_to_vector.
        """
        x, y = vector
        tilt_rad = math.atan2(math.sqrt(x**2 + y**2), 1.0)
        azimuth_rad = math.atan2(y, x)
        tilt_deg = math.degrees(tilt_rad)
        azimuth_deg = math.degrees(azimuth_rad)
        azimuth_deg = (azimuth_deg + 360) % 360  # Normalize
        return (tilt_deg, azimuth_deg)


if __name__ == "__main__":
    # Test tilt_azimuth_to_vector and tilt_vector_to_tilt_azimuth
    controller = Controller(None, live_output=False)
    # Example tilt and azimuth angles
    tilt_deg = 1.2
    azimuth_deg = 45.0
    vector = controller.tilt_azimuth_to_vector(tilt_deg, azimuth_deg)
    print(f"Vector from tilt {tilt_deg}° and azimuth {azimuth_deg}°: {vector}")
    tilt_deg_out, azimuth_deg_out = controller.tilt_vector_to_tilt_azimuth(vector)
    print(
        f"Converted back to tilt {tilt_deg_out:.2f}° and azimuth {azimuth_deg_out:.2f}°"
    )

    # Automatically check all combinations of tilt and azimuth
    for tilt in range(1, 13, 1):  # 0 to 90 degrees in steps of 10
        tilt_deg = tilt * 0.1
        for azimuth in range(0, 359, 10):  # 0 to 360 degrees in steps of 10
            vector = controller.tilt_azimuth_to_vector(tilt_deg, azimuth)
            tilt_out, azimuth_out = controller.tilt_vector_to_tilt_azimuth(vector)
            assert (
                abs(tilt_deg - tilt_out) < 0.1 and abs(azimuth - azimuth_out) < 0.1
            ), f"Mismatch for tilt {tilt_deg}° and azimuth {azimuth}°: got {tilt_out:.2f}°, {azimuth_out:.2f}°"
    print("All tilt/azimuth conversions passed successfully!")
