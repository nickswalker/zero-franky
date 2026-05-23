from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import uuid


class ProtocolError(ValueError):
    pass


class UnsupportedMotionType(TypeError):
    pass


MOTION_ENCODERS = {}


def motion_encoder(type_name: str):
    def register(fn):
        MOTION_ENCODERS[type_name] = fn
        return fn

    return register


@dataclass(frozen=True)
class RpcRequest:
    id: str
    method: str
    params: dict[str, Any]

    @classmethod
    def create(cls, method: str, params: dict[str, Any] | None = None) -> "RpcRequest":
        return cls(id=uuid.uuid4().hex, method=method, params=params or {})


def encode_affine(value: Any) -> dict[str, Any]:
    matrix = getattr(value, "matrix", None)
    if matrix is None:
        raise ProtocolError(f"Cannot encode {type(value).__name__} as Affine")
    return {
        "type": "Affine",
        "matrix": [[float(item) for item in row] for row in matrix],
    }


def encode_reference_type(value: Any) -> str:
    name = getattr(value, "name", None)
    if name in {"Relative", "Absolute"}:
        return name
    text = str(value)
    if text.endswith(".Relative"):
        return "Relative"
    if text.endswith(".Absolute"):
        return "Absolute"
    raise ProtocolError(f"Cannot encode {value!r} as ReferenceType")


def encode_relative_dynamics_factor(value: Any) -> dict[str, Any]:
    if isinstance(value, (int, float)):
        return {"velocity": float(value), "acceleration": float(value), "jerk": float(value)}
    return {
        "velocity": float(value.velocity),
        "acceleration": float(value.acceleration),
        "jerk": float(value.jerk),
    }


def encode_duration(value: Any | None) -> int | None:
    if value is None:
        return None
    return int(value.to_msec())


def encode_vector(value: Any | None) -> list[float] | None:
    if value is None:
        return None
    return [float(item) for item in value]


def encode_optional_float_vector(value: Any | None) -> list[float | None] | None:
    if value is None:
        return None
    return [None if item is None else float(item) for item in value]


def encode_cartesian_target(value: Any) -> dict[str, Any]:
    if getattr(value, "matrix", None) is not None:
        return {"type": "Affine", "value": encode_affine(value)}
    if hasattr(value, "end_effector_pose"):
        return {
            "type": "RobotPose",
            "end_effector_pose": encode_affine(value.end_effector_pose),
        }
    if hasattr(value, "pose"):
        pose = value.pose
        return {
            "type": "CartesianState",
            "pose": {
                "end_effector_pose": encode_affine(pose.end_effector_pose),
            },
        }
    raise ProtocolError(f"Cannot encode {type(value).__name__} as a cartesian target")


def encode_twist(value: Any) -> dict[str, Any]:
    return {
        "type": "Twist",
        "linear": [float(item) for item in value.linear],
        "angular": [float(item) for item in value.angular],
    }


def encode_robot_velocity(value: Any) -> dict[str, Any]:
    if hasattr(value, "end_effector_twist"):
        return {
            "type": "RobotVelocity",
            "end_effector_twist": encode_twist(value.end_effector_twist),
            "elbow_velocity": value.elbow_velocity,
        }
    if hasattr(value, "linear") and hasattr(value, "angular"):
        return {"type": "Twist", "value": encode_twist(value)}
    raise ProtocolError(f"Cannot encode {type(value).__name__} as a robot velocity")


def encode_joint_target(value: Any) -> dict[str, Any]:
    position = getattr(value, "position", value)
    return {"type": "JointState", "position": [float(item) for item in position]}


def encode_position_waypoint(waypoint: Any, target_encoder) -> dict[str, Any]:
    return {
        "target": target_encoder(waypoint.target),
        "reference_type": encode_reference_type(waypoint.reference_type),
        "relative_dynamics_factor": encode_relative_dynamics_factor(waypoint.relative_dynamics_factor),
        "minimum_time": encode_duration(waypoint.minimum_time),
        "hold_target_duration": encode_duration(waypoint.hold_target_duration),
        "max_total_duration": encode_duration(waypoint.max_total_duration),
    }


def encode_velocity_waypoint(waypoint: Any, target_encoder) -> dict[str, Any]:
    return {
        "target": target_encoder(waypoint.target),
        "relative_dynamics_factor": encode_relative_dynamics_factor(waypoint.relative_dynamics_factor),
        "minimum_time": encode_duration(waypoint.minimum_time),
        "hold_target_duration": encode_duration(waypoint.hold_target_duration),
        "max_total_duration": encode_duration(waypoint.max_total_duration),
    }


def encode_joint_impedance_fields(motion: Any) -> dict[str, Any]:
    return {
        "target": encode_vector(motion.target),
        "target_velocity": encode_vector(motion.target_velocity),
        "stiffness": encode_vector(motion.stiffness),
        "damping": encode_vector(motion.damping),
        "constant_torque_offset": encode_vector(motion.constant_torque_offset),
        "lower_joint_limits": encode_vector(motion.lower_joint_limits),
        "upper_joint_limits": encode_vector(motion.upper_joint_limits),
        "compensate_coriolis": bool(motion.compensate_coriolis),
        "max_delta_tau": float(motion.max_delta_tau),
        "joint_limit_activation_distance": float(motion.joint_limit_activation_distance),
        "joint_limit_stiffness": float(motion.joint_limit_stiffness),
        "joint_limit_damping": float(motion.joint_limit_damping),
        "joint_limit_max_torque": float(motion.joint_limit_max_torque),
    }


def encode_cartesian_impedance_fields(motion: Any) -> dict[str, Any]:
    return {
        "target": encode_affine(motion.target),
        "target_type": encode_reference_type(motion.target_type),
        "translational_stiffness": float(motion.translational_stiffness),
        "rotational_stiffness": float(motion.rotational_stiffness),
        "force_constraints": encode_optional_float_vector(motion.force_constraints),
        "nullspace_target": encode_vector(motion.nullspace_target),
        "nullspace_stiffness": float(motion.nullspace_stiffness),
        "max_delta_tau": float(motion.max_delta_tau),
        "lower_joint_limits": encode_vector(motion.lower_joint_limits),
        "upper_joint_limits": encode_vector(motion.upper_joint_limits),
        "joint_limit_activation_distance": float(motion.joint_limit_activation_distance),
        "joint_limit_stiffness": float(motion.joint_limit_stiffness),
        "joint_limit_damping": float(motion.joint_limit_damping),
        "joint_limit_max_torque": float(motion.joint_limit_max_torque),
        "translational_error_clip": encode_vector(motion.translational_error_clip),
        "rotational_error_clip": encode_vector(motion.rotational_error_clip),
    }


def encode_motion(motion: Any) -> dict[str, Any]:
    name = type(motion).__name__
    try:
        return MOTION_ENCODERS[name](motion)
    except KeyError as exc:
        raise UnsupportedMotionType(f"Unsupported motion type: {name}") from exc


@motion_encoder("CartesianMotion")
def encode_cartesian_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "CartesianMotion",
        "target": encode_cartesian_target(motion.target),
        "reference_type": encode_reference_type(motion.reference_type),
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
        "return_when_finished": bool(motion.return_when_finished),
        "ee_frame": encode_affine(motion.ee_frame) if getattr(motion, "ee_frame", None) is not None else None,
    }


@motion_encoder("CartesianWaypointMotion")
def encode_cartesian_waypoint_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "CartesianWaypointMotion",
        "waypoints": [encode_position_waypoint(waypoint, encode_cartesian_target) for waypoint in motion.waypoints],
        "ee_frame": encode_affine(motion.ee_frame) if getattr(motion, "ee_frame", None) is not None else None,
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
        "return_when_finished": bool(motion.return_when_finished),
    }


@motion_encoder("CartesianStopMotion")
def encode_cartesian_stop_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "CartesianStopMotion",
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
    }


@motion_encoder("JointMotion")
def encode_joint_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "JointMotion",
        "target": encode_joint_target(motion.target),
        "reference_type": encode_reference_type(motion.reference_type),
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
        "return_when_finished": bool(motion.return_when_finished),
    }


@motion_encoder("JointWaypointMotion")
def encode_joint_waypoint_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "JointWaypointMotion",
        "waypoints": [encode_position_waypoint(waypoint, encode_joint_target) for waypoint in motion.waypoints],
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
        "return_when_finished": bool(motion.return_when_finished),
    }


@motion_encoder("JointStopMotion")
def encode_joint_stop_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "JointStopMotion",
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
    }


@motion_encoder("CartesianVelocityMotion")
def encode_cartesian_velocity_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "CartesianVelocityMotion",
        "target": encode_robot_velocity(motion.target),
        "duration": encode_duration(motion.duration),
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
        "ee_frame": encode_affine(motion.ee_frame) if getattr(motion, "ee_frame", None) is not None else None,
    }


@motion_encoder("CartesianVelocityWaypointMotion")
def encode_cartesian_velocity_waypoint_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "CartesianVelocityWaypointMotion",
        "waypoints": [encode_velocity_waypoint(waypoint, encode_robot_velocity) for waypoint in motion.waypoints],
        "ee_frame": encode_affine(motion.ee_frame) if getattr(motion, "ee_frame", None) is not None else None,
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
    }


@motion_encoder("CartesianVelocityStopMotion")
def encode_cartesian_velocity_stop_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "CartesianVelocityStopMotion",
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
    }


@motion_encoder("JointVelocityMotion")
def encode_joint_velocity_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "JointVelocityMotion",
        "target": [float(item) for item in motion.target],
        "duration": encode_duration(motion.duration),
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
    }


@motion_encoder("JointVelocityWaypointMotion")
def encode_joint_velocity_waypoint_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "JointVelocityWaypointMotion",
        "waypoints": [
            encode_velocity_waypoint(waypoint, lambda target: [float(item) for item in target])
            for waypoint in motion.waypoints
        ],
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
    }


@motion_encoder("JointVelocityStopMotion")
def encode_joint_velocity_stop_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "JointVelocityStopMotion",
        "relative_dynamics_factor": encode_relative_dynamics_factor(motion.relative_dynamics_factor),
    }


@motion_encoder("JointImpedanceMotion")
def encode_joint_impedance_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "JointImpedanceMotion",
        **encode_joint_impedance_fields(motion),
    }


@motion_encoder("CartesianImpedanceMotion")
def encode_cartesian_impedance_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "CartesianImpedanceMotion",
        **encode_cartesian_impedance_fields(motion),
        "duration": encode_duration(motion.duration),
        "return_when_finished": bool(motion.return_when_finished),
        "finish_wait_factor": float(motion.finish_wait_factor),
    }


@motion_encoder("ExponentialImpedanceMotion")
def encode_exponential_impedance_motion(motion: Any) -> dict[str, Any]:
    return {
        "type": "ExponentialImpedanceMotion",
        **encode_cartesian_impedance_fields(motion),
        "exponential_decay": float(motion.exponential_decay),
    }


def encode_callback_state(
    robot_state: Any,
    time_step: Any,
    rel_time: Any,
    abs_time: Any,
    control_signal: Any,
) -> dict[str, Any]:
    return {
        "q": [float(value) for value in robot_state.q],
        "dq": [float(value) for value in robot_state.dq],
        "O_T_EE": [[float(value) for value in row] for row in robot_state.O_T_EE.matrix],
        "time_step": float(time_step.to_sec()),
        "rel_time": float(rel_time.to_sec()),
        "abs_time": float(abs_time.to_sec()),
        "control_signal": _encode_control_signal(control_signal),
    }


def _encode_control_signal(control_signal: Any):
    if hasattr(control_signal, "q"):
        return {"type": "JointPositions", "q": [float(value) for value in control_signal.q]}
    if hasattr(control_signal, "position"):
        return {"type": "JointPositions", "q": [float(value) for value in control_signal.position]}
    return {"type": type(control_signal).__name__, "repr": repr(control_signal)}
