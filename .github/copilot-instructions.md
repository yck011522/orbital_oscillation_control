# Orbital Oscillation Control - AI Agent Guidelines

## Architecture Overview

This project is a **position control system** for a balance table with load sensors. The system detects the weight distribution via 4-channel load cells, estimates the object's center of pressure (CoP), and dynamically tilts the table using stepper motor-controlled actuators to maintain oscillatory motion.

### Core Data Flow
1. **Sensor Input** (`sensor_common.py`): Modbus load cells read pressure at 4 positions (CH1–CH4) → weighted CoP calculation
2. **Pose Estimation** (`pose_estimator.py`): CoP → angle (0–360°), velocity, motion state detection (stationary/oscillating/rotating)
3. **Control FSM** (`controller.py`): Motion state → control mode (start, pump, maintain rotation, decay) → tilt vector command
4. **Actuator Control** (`motor_control.py`, `actuator_utils.py`): Tilt angles (elevation, azimuth) → stepper motor positions via serial

### Critical Coordinate Systems
- **Angle Convention** (documented in `pose_estimator.py`): 0° = +X (East), 90° = +Y (North), counter-clockwise. Used consistently across pose estimation and control targeting.
- **Tilt Angles**: Elevation (0–~1.2°) = table tilt magnitude; Azimuth (0–360°) = tilt direction. Converted to X/Y components via `polar_to_tilt_components()`.
- **Actuator Space**: 4 stepper motors control elevation via IK lookup table (`actuator_ik_table05.py`). Soft limits: 0.1–49.0 mm travel.

## Key File Purposes

| File | Role |
|------|------|
| `sensor_common.py` | Load cell discovery, Modbus communication, SLOPE/TARE/POSITIONS constants |
| `pose_estimator.py` | Threaded sensor stream → CoP → angle/velocity/motion state (1=stationary, 2=oscillating, 3=rotating) |
| `controller.py` | FSM-driven control loop (100 Hz default). Selects pump/rotation/decay strategies based on motion state. |
| `motor_control.py` | Serial motor commands (address 1–4), homing, soft limits |
| `actuator_utils.py` | Tilt angle math, IK lookup, rate limiting |
| `main.py` | Thread orchestration: PoseEstimator → Controller → Visualization |
| `timing_utils.py` | Frequency estimator (exponential smoothing for Hz monitoring) |

## Architectural Patterns

### Threading Model
- **Three daemon threads**: PoseEstimator, Controller, Visualizer. Each has its own loop frequency.
- **No explicit synchronization**: Threads communicate via `pose_estimator.state` dict (angle, velocity, motion_state).
- **Safe access**: `get_latest_state()` returns immutable dict snapshot.

### Control Modes (FSM in Controller)
1. **WAIT_WHILE_STATIONARY** (state=0): Wait 40s after object stops.
2. **PUMP** (state=2): Multi-phase oscillation control triggered from stationary:
   - Phase 1 (initial): High acceleration, short duration, builds oscillation
   - Phase 2 (regular): Lower acceleration, maintains oscillation across 95°–260° azimuth range
3. **FULL_ROTATION** (state=3): Tiny constant tilt (0.1°) to nudge rotation at 90° lead angle
4. **DECAY** (state=3→waiting): Graceful shutdown (zeroing control vector)

### Sensor Calibration
- `SLOPE[ch]`: Raw ADC counts → kg conversion (cal. per channel)
- `TARE[ch]`: Zero-load raw value (updated with load cells unloaded)
- `SENSOR_POSITIONS[ch]`: Physical XY position of each load cell (e.g., CH1 at 176.7, 64.3 mm)

## Common Workflows

### Adding a New Control Mode
1. Add state constant in `Controller.__init__` (e.g., `self.STATE_CUSTOM = 4`)
2. Implement `control_custom(self, state)` returning tuple `(tilt_deg, azimuth_deg)`
3. Add branch in `run()` FSM to call it and set next state
4. Test with `debug_force_state = 4` in `main.py`

### Tuning Motor Response
- **Speed**: `speed_rpm` parameter in `send_absolute_position_mm()` (default 200–300)
- **Soft limits**: Edit `soft_limit_min/max` (default 0.1–49.0 mm)
- **Homing reference**: Always call `home_all_motors(ser, settle_position_mm=28.1)` at startup

### Calibrating Load Cells
1. Run `manual_scripts/sensor_read_all_channels.py` with known weights
2. Solve TARE (zero-load ADC) and SLOPE (kg/count) via linear regression
3. Update `sensor_common.py` constants
4. Verify with weighted object at center → distance ≈ 0 mm

### Updating Pose Estimation Logic
- **CoP calculation**: Weighted average of sensor positions (see `pose_estimator.py`)
- **Motion state thresholds**: `velocity_threshold_start` (0.05 rad/s), reversal detection (sign change)
- **Angle smoothing**: `position_filter_tau` (0.2 s) exponential filter reduces jitter

## Project-Specific Conventions

### Rate Limiting
- Used in control loops to prevent jerky actuator motion.
- Pattern: `new_val = limit_change(current, target, max_delta_per_cycle)`
- Example: `filter_rate_elevation = 0.01 rad/cycle`, `filter_rate_azimuth = 0.9 rad/cycle`

### Angle Wraparound Handling
- Always normalize azimuth to [0, 360) via `angle % 360` after calculation
- Detect direction reversal: `if raw_diff > 180: raw_diff -= 360` (handle 359° → 0° crossover)

### Exponential Smoothing
- Frequency: `smoothed = alpha * instant + (1 - alpha) * smoothed_prev` (alpha ≈ 0.9 = heavy smoothing)
- Used for velocity/acceleration filtering and frequency monitoring

### Testing Strategy
- **Unit tests**: Angle conversions, IK validity (see `__main__` blocks in `controller.py`)
- **Integration**: Use recorded CSV data in non-live mode (`main.py` with `USE_LIVE_SYSTEM=False`)
- **Hardware**: Manual scripts in `manual_scripts/` for isolated motor/sensor testing

## Integration Points & Dependencies

| External | Usage |
|----------|-------|
| **pymodbus** | Modbus RTU over RS485 serial (load cells) |
| **PySerial** | Stepper motor command serial (TTL) |
| **OpenCV** | Real-time 2D polar visualization (angle/radius plot) |
| **NumPy/Pandas** | Math, CSV playback for replays |
| **Threading** | Native Python `threading` for concurrency |

### Serial Port Discovery
- **Motors**: Auto-scan COMx (or `/dev/ttyS0` on Raspberry Pi) via `motor_control.open_serial()`
- **Load Cells**: Auto-scan COMx via `sensor_common.find_modbus_device(baudrate=38400)`
- Both use context manager pattern: `with open_serial() as ser:`

## Debugging Tips

- **Frequency drops**: Check viz thread (OpenCV slow), adjust `control_freq`, or profile with `timing_utils.FrequencyEstimator`
- **Stuck motor**: Run `manual_scripts/speed_test_motor.py` to isolate hardware
- **Sensor noise**: Increase `position_filter_tau` or check TARE calibration
- **Angle jitter**: Verify sensor positions (SENSOR_POSITIONS) match physical layout; check load cell balance
- **Oscillation won't start**: Check `pump_max_tilt_initial` (0.70°), phase range (95°–260°), and `starting_acceleration_rate` (1.02 rad/s²)
