"""Shared defaults and velocity limits for the controller and TCP sender."""

import math


DEFAULT_HOST = "192.168.192.5"
MAX_LINEAR_SPEED = 0.2  # m/s, magnitude of the planar velocity vector
MAX_ANGULAR_SPEED = math.radians(5.0)  # API uses rad/s; limit is 5 degrees/s


def validate_limits(max_linear_speed, max_angular_speed):
    for name, value, ceiling in (
        ("max_linear_speed", max_linear_speed, MAX_LINEAR_SPEED),
        ("max_angular_speed", max_angular_speed, MAX_ANGULAR_SPEED),
    ):
        if not math.isfinite(value) or not 0.0 < value <= ceiling:
            raise ValueError(f"{name} must be finite and in (0, {ceiling}]")


def limit_velocity(vx, vy, w, max_linear_speed=MAX_LINEAR_SPEED,
                   max_angular_speed=MAX_ANGULAR_SPEED):
    """Preserve planar direction, saturate vector magnitude, clamp yaw rate."""
    validate_limits(max_linear_speed, max_angular_speed)
    if not all(math.isfinite(value) for value in (vx, vy, w)):
        raise ValueError("Velocity components must be finite")
    # Scale before taking the norm so even very large finite inputs stay valid.
    largest_component = max(abs(vx), abs(vy))
    if largest_component > 0.0:
        scaled_x, scaled_y = vx / largest_component, vy / largest_component
        scaled_norm = math.hypot(scaled_x, scaled_y)
        allowed_component = max_linear_speed / scaled_norm
        if largest_component > allowed_component:
            vx, vy = scaled_x * allowed_component, scaled_y * allowed_component
    w = max(-max_angular_speed, min(max_angular_speed, w))
    return vx, vy, w
