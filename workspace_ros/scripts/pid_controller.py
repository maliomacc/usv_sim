#!/usr/bin/env python3
"""
PID Controller Module for YILDIZ USV

Provides reusable PID controller class with:
- Anti-windup (integral clamping + back-calculation)
- Derivative filtering (low-pass for noise reduction)
- Output limits
- Reset functionality

Author: YILDIZ USV Team
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class PIDGains:
    """PID gain parameters."""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    integral_max: float = 1.0
    output_min: float = -1.0
    output_max: float = 1.0


class PIDController:
    """
    PID Controller with anti-windup and derivative filtering.

    Features:
    - Clamping anti-windup for integral term
    - Back-calculation anti-windup when output saturates
    - Low-pass filter on derivative term
    - Derivative-on-measurement option (reduces setpoint kick)

    Usage:
        pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        output = pid.compute(error, dt)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        integral_max: float = 1.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        derivative_alpha: float = 0.1,
        derivative_on_measurement: bool = False
    ):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            integral_max: Maximum absolute value for integral term (anti-windup)
            output_min: Minimum output value
            output_max: Maximum output value
            derivative_alpha: Low-pass filter coefficient for derivative (0-1, lower = more filtering)
            derivative_on_measurement: If True, compute derivative on measurement instead of error
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = abs(integral_max)
        self.output_min = output_min
        self.output_max = output_max
        self.derivative_alpha = np.clip(derivative_alpha, 0.01, 1.0)
        self.derivative_on_measurement = derivative_on_measurement

        # State variables
        self.integral = 0.0
        self.last_error = 0.0
        self.last_measurement = 0.0
        self.filtered_derivative = 0.0
        self.last_output = 0.0

        # For debugging/tuning
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0

    def compute(self, error: float, dt: float, measurement: Optional[float] = None) -> float:
        """
        Compute PID output.

        Args:
            error: Current error (setpoint - measurement)
            dt: Time step in seconds
            measurement: Current measurement (required if derivative_on_measurement=True)

        Returns:
            Control output (clamped to output limits)
        """
        if dt <= 0:
            dt = 0.001  # Prevent division by zero

        # --- Proportional term ---
        p_term = self.kp * error

        # --- Integral term with clamping anti-windup ---
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        i_term = self.ki * self.integral

        # --- Derivative term ---
        if self.derivative_on_measurement and measurement is not None:
            # Derivative on measurement (avoids setpoint kick)
            raw_derivative = -(measurement - self.last_measurement) / dt
            self.last_measurement = measurement
        else:
            # Derivative on error
            raw_derivative = (error - self.last_error) / dt

        # Low-pass filter for derivative (reduces noise)
        self.filtered_derivative = (
            self.derivative_alpha * raw_derivative +
            (1.0 - self.derivative_alpha) * self.filtered_derivative
        )
        d_term = self.kd * self.filtered_derivative

        # --- Compute raw output ---
        raw_output = p_term + i_term + d_term

        # --- Clamp output ---
        output = np.clip(raw_output, self.output_min, self.output_max)

        # --- Back-calculation anti-windup ---
        # If output is saturated, reduce integral to prevent windup
        if raw_output != output and self.ki != 0:
            saturation_error = output - raw_output
            self.integral += saturation_error / self.ki * 0.5  # Partial back-calculation
            self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)

        # Store for next iteration
        self.last_error = error
        self.last_output = output

        # Store terms for debugging
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term

        return output

    def reset(self):
        """Reset controller state (call when switching modes or on large setpoint changes)."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_measurement = 0.0
        self.filtered_derivative = 0.0
        self.last_output = 0.0

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains (can be called during runtime for adaptive control)."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_output_limits(self, output_min: float, output_max: float):
        """Update output limits."""
        self.output_min = output_min
        self.output_max = output_max

    def get_terms(self) -> Tuple[float, float, float]:
        """Get individual P, I, D terms for debugging/tuning."""
        return self.last_p_term, self.last_i_term, self.last_d_term

    def get_integral(self) -> float:
        """Get current integral value for monitoring."""
        return self.integral


class HeadingPIDController(PIDController):
    """
    Specialized PID controller for heading/yaw control.

    Handles angle wrapping automatically.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def compute_heading(self, current_heading: float, target_heading: float, dt: float) -> float:
        """
        Compute angular velocity command for heading control.

        Args:
            current_heading: Current heading in radians
            target_heading: Desired heading in radians
            dt: Time step in seconds

        Returns:
            Angular velocity command (rad/s)
        """
        # Calculate heading error with proper wrapping
        error = self.normalize_angle(target_heading - current_heading)
        return self.compute(error, dt, measurement=current_heading)


class VelocityPIDController(PIDController):
    """
    Specialized PID controller for velocity control.

    Includes feedforward term for better tracking.
    """

    def __init__(self, feedforward_gain: float = 0.0, **kwargs):
        super().__init__(**kwargs)
        self.feedforward_gain = feedforward_gain

    def compute_velocity(
        self,
        current_velocity: float,
        target_velocity: float,
        dt: float
    ) -> float:
        """
        Compute thrust/acceleration command for velocity control.

        Args:
            current_velocity: Current velocity (m/s)
            target_velocity: Desired velocity (m/s)
            dt: Time step in seconds

        Returns:
            Thrust/acceleration command
        """
        error = target_velocity - current_velocity

        # Feedforward term (helps with steady-state tracking)
        feedforward = self.feedforward_gain * target_velocity

        # PID feedback
        feedback = self.compute(error, dt, measurement=current_velocity)

        return feedforward + feedback


def create_heading_pid(
    kp: float = 1.2,
    ki: float = 0.05,
    kd: float = 0.3,
    max_angular_vel: float = 1.5
) -> HeadingPIDController:
    """Factory function to create a heading PID controller with sensible defaults."""
    return HeadingPIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        integral_max=1.0,
        output_min=-max_angular_vel,
        output_max=max_angular_vel,
        derivative_alpha=0.2,
        derivative_on_measurement=True
    )


def create_velocity_pid(
    kp: float = 0.8,
    ki: float = 0.02,
    kd: float = 0.1,
    max_accel: float = 2.0,
    feedforward: float = 0.5
) -> VelocityPIDController:
    """Factory function to create a velocity PID controller with sensible defaults."""
    return VelocityPIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        integral_max=0.5,
        output_min=-max_accel,
        output_max=max_accel,
        derivative_alpha=0.3,
        feedforward_gain=feedforward
    )


# For standalone testing
def main():
    """Test PID controller."""
    import time

    # Create heading controller
    heading_pid = create_heading_pid()

    # Simulate heading control
    current = 0.0
    target = np.pi / 2  # 90 degrees
    dt = 0.05

    print("Heading PID Test:")
    print(f"Target: {np.degrees(target):.1f} deg")
    print("-" * 40)

    for i in range(50):
        angular_vel = heading_pid.compute_heading(current, target, dt)
        current += angular_vel * dt  # Simple integration
        current = HeadingPIDController.normalize_angle(current)

        if i % 10 == 0:
            error = np.degrees(target - current)
            p, i_term, d = heading_pid.get_terms()
            print(f"Step {i:3d}: heading={np.degrees(current):6.1f}deg, "
                  f"error={error:6.1f}deg, cmd={angular_vel:5.2f}")

    print("\nFinal heading:", np.degrees(current), "deg")
    print("Target was:", np.degrees(target), "deg")


if __name__ == '__main__':
    main()
