"""Planar control and rigid transforms, independent of ROS."""

from dataclasses import dataclass
import math

from robokit_control.limits import MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, limit_velocity, validate_limits


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalized_quaternion(values):
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("Quaternion must contain four finite xyzw values")
    norm = math.hypot(*values)
    if norm < 1e-8 or not math.isfinite(norm):
        raise ValueError("Quaternion has invalid norm")
    return tuple(value / norm for value in values)


def multiply_quaternion(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    result = multiply_quaternion(
        multiply_quaternion(quaternion, (*vector, 0.0)), (-x, -y, -z, w)
    )
    return result[:3]


def yaw_from_quaternion(quaternion):
    x, y, z, w = normalized_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def transform_pose(position, orientation, world_rotation, world_translation,
                   body_translation, body_rotation):
    """T_control_base = T_control_mocap * T_mocap_body * T_body_base."""
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        raise ValueError("Position must contain three finite values")
    orientation = normalized_quaternion(orientation)
    body_in_world = multiply_quaternion(world_rotation, orientation)
    body_origin = rotate_vector(world_rotation, position)
    base_offset = rotate_vector(body_in_world, body_translation)
    base_position = tuple(
        body_origin[i] + world_translation[i] + base_offset[i] for i in range(3)
    )
    if not all(math.isfinite(value) for value in base_position):
        raise ValueError("Transformed position is not finite")
    base_orientation = normalized_quaternion(multiply_quaternion(body_in_world, body_rotation))
    return base_position, base_orientation


@dataclass(frozen=True)
class ControlConfig:
    kp_xy: float = 0.8
    kp_yaw: float = 1.2
    max_linear_speed: float = MAX_LINEAR_SPEED
    max_angular_speed: float = MAX_ANGULAR_SPEED
    max_linear_acceleration: float = 0.3
    max_angular_acceleration: float = 0.8
    position_tolerance: float = 0.01
    yaw_tolerance: float = math.pi / 180.0
    settle_time: float = 0.5

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        validate_limits(self.max_linear_speed, self.max_angular_speed)


class PoseController:
    def __init__(self, config):
        self.config = config
        self.goal = None
        self.within_tolerance_since = None
        self.previous_velocity = (0.0, 0.0, 0.0)

    def set_goal(self, goal):
        if len(goal) != 3 or not all(math.isfinite(value) for value in goal):
            raise ValueError("Goal must contain finite x, y and yaw")
        self.goal = tuple(goal)
        self.within_tolerance_since = None

    def cancel(self):
        self.goal = None
        self.within_tolerance_since = None
        self.previous_velocity = (0.0, 0.0, 0.0)

    def step(self, pose, now, dt):
        if self.goal is None:
            return (0.0, 0.0, 0.0), "idle"
        if not all(math.isfinite(value) for value in (*pose, now, dt)) or dt <= 0:
            raise ValueError("Invalid pose or control time")
        x, y, yaw = pose
        goal_x, goal_y, goal_yaw = self.goal
        dx, dy = goal_x - x, goal_y - y
        yaw_error = wrap_angle(goal_yaw - yaw)
        distance = math.hypot(dx, dy)
        cfg = self.config
        if distance <= cfg.position_tolerance and abs(yaw_error) <= cfg.yaw_tolerance:
            self.previous_velocity = (0.0, 0.0, 0.0)
            if self.within_tolerance_since is None:
                self.within_tolerance_since = now
            if now - self.within_tolerance_since >= cfg.settle_time:
                self.cancel()
                return (0.0, 0.0, 0.0), "reached"
            return (0.0, 0.0, 0.0), "settling"
        self.within_tolerance_since = None
        speed = min(cfg.kp_xy * distance, cfg.max_linear_speed)
        if distance <= cfg.position_tolerance:
            speed = 0.0
        direction = math.atan2(dy, dx) - yaw
        vx, vy = speed * math.cos(direction), speed * math.sin(direction)
        w = max(-cfg.max_angular_speed, min(cfg.max_angular_speed, cfg.kp_yaw * yaw_error))
        if abs(yaw_error) <= cfg.yaw_tolerance:
            w = 0.0
        old_vx, old_vy, old_w = self.previous_velocity
        delta_x, delta_y = vx - old_vx, vy - old_vy
        delta_speed = math.hypot(delta_x, delta_y)
        allowed_change = cfg.max_linear_acceleration * dt
        scale = allowed_change / max(allowed_change, delta_speed)
        velocity = (
            old_vx + delta_x * scale,
            old_vy + delta_y * scale,
            old_w + max(-cfg.max_angular_acceleration * dt,
                        min(cfg.max_angular_acceleration * dt, w - old_w)),
        )
        velocity = limit_velocity(*velocity, cfg.max_linear_speed, cfg.max_angular_speed)
        self.previous_velocity = velocity
        return velocity, "tracking"
