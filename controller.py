import threading
import time
from motor_control import send_absolute_position_mm, open_serial, home_all_motors
from actuator_utils import actuator_length, polar_to_tilt_components

from timing_utils import FrequencyEstimator
from pose_estimator import PoseEstimator
import math

class Controller(threading.Thread):
    def __init__(self, pose_estimator: PoseEstimator, control_freq=100):
        super().__init__(daemon=True, name="Controller")
        self.pose_estimator = pose_estimator
        self.control_freq = control_freq
        self.stop_event = threading.Event()

        # FSM states
        self.STATE_WAIT_TILL_STATIONARY = 0
        self.STATE_DELAY_BEFORE_PUMP = 1
        self.STATE_PUMP = 2
        self.STATE_DECAY = 3

        self.state = self.STATE_WAIT_TILL_STATIONARY
        self.debug_force_state = 2  # set this to 2 (PUMP) to force debug mode

        # Timing and parameters
        self.delay_timer_start = None
        self.full_rotation_start_time = time.time()
        self.delay_duration = 2.0
        self.pump_duration = 10.0

        # Control function registry (based on pose motion_state)
        self.control_functions = {
            1: self.control_stationary,
            2: self.control_oscillation,
            3: self.control_full_rotation
        }

        # Control policy parameters
        self._current_tilt = 0.0
        self._current_azimuth = 0.0 

        # You can populate these via slider/UI later
        self.phase_start = 80
        self.phase_end = 190
        self.max_tilt = 0.80  # degrees
        self.acceleration_rate = 1.0  # deg/sec²
        self.deceleration_rate = 1.0  # deg/sec²
        self.lead_angle_deg = 60.0  # lead angle for azimuth

        # For full rotation control
        self.full_rotation_tilt = 0.45         # degrees (constant tilt to maintain)
        self.full_rotation_lead_angle = 15.0   # degrees ahead of current object angle

        # Serial connection for motor control
        self.ser = open_serial()
        if not home_all_motors(self.ser, settle_position_mm=28.1):
            print("⚠️ Homing failed. Exiting controller.")
            exit()

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

            state = self.pose_estimator.get_latest_state()

            # Check debug override
            if self.debug_force_state is not None:
                active_state = self.debug_force_state
            else:
                active_state = self.state

            # FSM Logic
            if active_state == self.STATE_WAIT_TILL_STATIONARY:
                if state["motion_state"] == 1:  # stationary
                    self.delay_timer_start = time.time()
                    self.state = self.STATE_DELAY_BEFORE_PUMP

            elif active_state == self.STATE_DELAY_BEFORE_PUMP:
                if time.time() - self.delay_timer_start > self.delay_duration:
                    self.state = self.STATE_PUMP

            elif active_state == self.STATE_PUMP:
                pose_state = state["motion_state"]
                if pose_state in self.control_functions:
                    target_tilt, target_azimuth = self.control_functions[pose_state](state)
                    print(f"Control Target Tilt: {target_tilt:.2f} degrees, Azimuth: {target_azimuth:.2f} degrees")
                    # Send target to actuator here
                    self.send_target_to_motor(target_tilt, target_azimuth)

                    
                # Logic to transition to decay state
                if state["motion_state"] == 3:  # Full rotation reached
                    if time.time() - self.full_rotation_start_time > self.pump_duration:
                        self.state = self.STATE_DECAY
                else:
                    self.full_rotation_start_time = time.time()

            elif active_state == self.STATE_DECAY:
                # Optional: graceful decay logic
                if state["motion_state"] == 1:  # back to stationary
                    self.state = self.STATE_WAIT_TILL_STATIONARY

            self.freq_estimator.update()
        
        send_absolute_position_mm(self.ser, 21.5, 0, speed_rpm=200)
        self.ser.close()


    def stop(self):
        self.stop_event.set()

    # === Control Functions ===
    def control_stationary(self, state):
        return 0.0 , 0.0  # No movement

    def control_oscillation(self, state):
        """
        Calculates target tilt and azimuth based on oscillation phase.
        This function is called during the PUMP controller state.
        """

        phase_deg = state["phase"]
        obj_angle_deg = state["angle"]

        # === TILT CONTROL ===
        in_active_range = self.phase_start <= phase_deg <= self.phase_end

        if in_active_range:
            # Accelerate upward
            print("Accelerate upward")
            self._current_tilt += self.acceleration_rate * self.delta_time
            self._current_tilt = min(self._current_tilt, self.max_tilt)
        else:
            # Decelerate downward
            print("Decelerate downward")
            self._current_tilt -= self.deceleration_rate * self.delta_time
            self._current_tilt = max(self._current_tilt, 0.0)

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
        return self._current_tilt, azimuth_deg


    def control_full_rotation(self, state):
        """
        Maintains a constant tilt and points azimuth slightly ahead of the object
        to sustain rotation.
        """

        obj_angle_deg = state["angle"]
        direction_is_forward = self.pose_estimator.get_direction()  # True for CW

        # === TILT ===
        self._current_tilt = self.full_rotation_tilt  # hold constant

        # === AZIMUTH ===
        if direction_is_forward:
            azimuth_deg = obj_angle_deg + self.full_rotation_lead_angle
        else:
            azimuth_deg = obj_angle_deg - self.full_rotation_lead_angle

        # Normalize azimuth to [0, 360]
        azimuth_deg = (azimuth_deg + 360) % 360

        return  self._current_tilt, azimuth_deg
    
    def send_target_to_motor(self, target_tilt, target_azimuth):
        now = time.time()
        dt = now - self.last_motor_time
        self.last_motor_time = now

        # Convert to radians
        tilt_rad = math.radians(target_tilt)
        azimuth_rad = math.radians(target_azimuth)
        theta_x, theta_y = polar_to_tilt_components(tilt_rad, azimuth_rad)

        L_x_plus  = actuator_length(theta_x) + self.actuator_value_offset
        L_x_minus = actuator_length(-theta_x) + self.actuator_value_offset
        L_y_plus  = actuator_length(theta_y) + self.actuator_value_offset
        L_y_minus = actuator_length(-theta_y) + self.actuator_value_offset

        target_positions = {
            1: L_x_plus,
            2: L_y_minus,
            3: L_x_minus,
            4: L_y_plus
        }

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
        send_absolute_position_mm(self.ser, L_x_plus, 1, speed_rpm=estimated_rpm[1])
        send_absolute_position_mm(self.ser, L_y_minus, 2, speed_rpm=estimated_rpm[2])
        send_absolute_position_mm(self.ser, L_x_minus, 3, speed_rpm=estimated_rpm[3])
        send_absolute_position_mm(self.ser, L_y_plus, 4, speed_rpm=estimated_rpm[4])

        self.prev_positions = target_positions
