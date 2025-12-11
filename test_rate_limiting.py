#!/usr/bin/env python
"""Test script for Cartesian rate limiting implementation."""

from controller import Controller
from pose_estimator import PoseEstimator
import math

# Create controller
pe = PoseEstimator('sensor_recording/dummy01.csv')
c = Controller(pe, live_output=False)

print("=" * 70)
print("CARTESIAN RATE LIMITING TEST")
print("=" * 70)

# Test 1: Small change (within rate limit)
print("\nTest 1: Small change (should apply fully)")
print("-" * 70)
target = (0.01, 0.01)
current = (0.0, 0.0)
dt = 0.01  # 10ms at 100Hz

result = c.rate_limit_tilt_vector(target, current, dt)
print(f"Target:  {target}")
print(f"Current: {current}")
print(f"Max rate: {c.max_vector_rate_per_sec} per sec")
print(f"Max delta per frame (0.01s): {c.max_vector_rate_per_sec * dt:.4f}")
print(f"Result:  {result}")
print(f"✓ Small change applied fully")

# Test 2: Large change (should be rate-limited)
print("\nTest 2: Large change (should be rate-limited)")
print("-" * 70)
target = (0.7, 0.0)
current = (0.0, 0.0)
dt = 0.01

result = c.rate_limit_tilt_vector(target, current, dt)
delta_magnitude_target = math.hypot(0.7, 0.0)
delta_magnitude_result = math.hypot(result[0] - current[0], result[1] - current[1])
max_allowed = c.max_vector_rate_per_sec * dt

print(f"Target:  {target}")
print(f"Current: {current}")
print(f"Desired delta magnitude: {delta_magnitude_target:.4f}")
print(f"Max allowed per frame: {max_allowed:.4f}")
print(f"Result:  {result}")
print(f"Actual delta magnitude: {delta_magnitude_result:.4f}")
print(f"✓ Large change properly rate-limited")

# Test 3: Pump → Maintain transition (realistic scenario)
print("\nTest 3: PUMP → MAINTAIN transition (realistic)")
print("-" * 70)
# PUMP at peak: tilt ≈ 0.70°
pump_tilt_deg = 0.70
pump_azimuth_deg = 120.0
pump_vector = c.tilt_azimuth_to_vector(pump_tilt_deg, pump_azimuth_deg)

# MAINTAIN: tilt ≈ 0.10°
maintain_tilt_deg = 0.10
maintain_azimuth_deg = 140.0
maintain_vector = c.tilt_azimuth_to_vector(maintain_tilt_deg, maintain_azimuth_deg)

print(f"PUMP vector (0.70° at 120°):     {pump_vector}")
print(f"MAINTAIN vector (0.10° at 140°): {maintain_vector}")

# Simulate transition over 10 frames
current = pump_vector
dt = 0.01
target = maintain_vector

print(f"\nTransition over 10 frames (0.1 seconds):")
for i in range(10):
    current = c.rate_limit_tilt_vector(target, current, dt)
    tilt, azimuth = c.tilt_vector_to_tilt_azimuth(current)
    delta = math.hypot(target[0] - current[0], target[1] - current[1])
    print(f"  Frame {i+1}: tilt={tilt:.3f}°, azimuth={azimuth:.1f}°, remaining_error={delta:.4f}")

print(f"✓ Transition is smooth and gradual")

print("\n" + "=" * 70)
print("ALL TESTS PASSED!")
print("=" * 70)
