from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Any
import uuid


class ProtocolError(ValueError):
    pass


class UnsupportedMotionType(TypeError):
    pass


class RpcError(RuntimeError):
    """Raised on the client when a remote zero_franky RPC call fails."""


def format_exception(exc: BaseException) -> str:
    """Render an exception as `TypeName: message`, without stdlib repr quirks.

    KeyError.__str__ reprs its argument (so `KeyError("Unknown robot id: x")`
    stringifies as `"Unknown robot id: x"` with the quotes included) - special
    case it so messages crossing the RPC boundary read the same as they were
    raised, instead of picking up stray quoting.
    """
    message = str(exc.args[0]) if isinstance(exc, KeyError) and exc.args else str(exc)
    return f"{type(exc).__name__}: {message}" if message else type(exc).__name__


#: Wire spelling of `franky.CRITICAL`: unpin damping. Distinct from a null
#: damping, which means the caller did not name damping at all.
CRITICAL_DAMPING = "critical"


def is_critical_damping(value: Any) -> bool:
    """Whether a wire value is the unpin sentinel."""
    return isinstance(value, str) and value == CRITICAL_DAMPING


def decode_damping(value: Any) -> Any:
    """Resolve a wire damping for franky's gains objects.

    The sentinel and a null damping both mean unpinned; they differ only in
    whether the call acts at all, which the caller settles first.
    """
    if isinstance(value, str):
        if not is_critical_damping(value):
            raise ProtocolError(f"Unknown damping sentinel: {value!r}")
        return None
    return value


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


def encode_friction_compensation_params(value: Any | None) -> dict[str, Any] | None:
    if value is None:
        return None
    return {
        "coulomb": encode_vector(value.coulomb),
        "viscous": encode_vector(value.viscous),
        "max_torque": encode_vector(value.max_torque),
        "velocity_epsilon": float(value.velocity_epsilon),
    }


def encode_matrix(value: Any | None) -> list[list[float]] | None:
    if value is None:
        return None
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        value = tolist()
    return [[float(item) for item in row] for row in value]


def encode_cartesian_diag(value: Any | None) -> list[float] | None:
    if value is None:
        return None
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        value = tolist()
    rows = [list(row) if isinstance(row, (list, tuple)) else row for row in value]
    if len(rows) == 6 and all(isinstance(row, (list, tuple)) for row in rows):
        return [float(rows[i][i]) for i in range(6)]
    return [float(item) for item in value]


def encode_cartesian_gains(value: Any) -> dict[str, Any]:
    if hasattr(value, "stiffness"):
        return {
            "stiffness": encode_matrix(value.stiffness),
            "damping": encode_matrix(value.damping),
        }
    return {
        "translational_stiffness": float(value.translational_stiffness),
        "rotational_stiffness": float(value.rotational_stiffness),
    }


def encode_nullspace_gains(value: Any) -> dict[str, Any]:
    return {
        "posture_stiffness": encode_rpc_value(value.posture_stiffness),
        "posture_damping": encode_rpc_value(value.posture_damping),
        "posture_max_torque": None if value.posture_max_torque is None else float(value.posture_max_torque),
        "manipulability_gain": float(value.manipulability_gain),
        "manipulability_damping": float(value.manipulability_damping),
        "manipulability_max_torque": (
            None if value.manipulability_max_torque is None else float(value.manipulability_max_torque)
        ),
    }


def encode_rpc_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, bytes, bool, int, float)):
        return value
    if isinstance(value, dict):
        return {str(key): encode_rpc_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [encode_rpc_value(item) for item in value]
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return encode_rpc_value(tolist())
    item = getattr(value, "item", None)
    if callable(item):
        return encode_rpc_value(item())
    # franky's CRITICAL, matched by type name so the client stays importable
    # without franky.
    if type(value).__name__ == "_CriticalDamping":
        return CRITICAL_DAMPING
    if type(value).__name__ == "FrictionCompensationParams":
        return encode_friction_compensation_params(value)
    if type(value).__name__ == "CartesianImpedanceGains":
        return encode_cartesian_gains(value)
    if type(value).__name__ == "JointImpedanceGains":
        return {
            "stiffness": encode_rpc_value(value.stiffness),
            "damping": encode_rpc_value(value.damping),
        }
    if type(value).__name__ == "NullspaceGains":
        return encode_nullspace_gains(value)
    if type(value).__name__ == "PostureTask":
        return {
            "target": encode_vector(value.target),
            "stiffness": encode_rpc_value(value.stiffness),
            "damping": encode_rpc_value(value.damping),
            "max_torque": None if value.max_torque is None else float(value.max_torque),
        }
    if type(value).__name__ == "ManipulabilityTask":
        return {
            "gain": float(value.gain),
            "damping": float(value.damping),
            "max_torque": None if value.max_torque is None else float(value.max_torque),
        }
    if type(value).__name__ == "JointReference":
        return {
            "type": "JointReference",
            "q": encode_vector(value.q),
            "dq": encode_vector(value.dq),
            "tau_ff": encode_vector(value.tau_ff),
        }
    if type(value).__name__ == "CartesianReference":
        target_twist = getattr(value, "target_twist", None)
        target_acceleration = getattr(value, "target_acceleration", None)
        return {
            "type": "CartesianReference",
            "target": encode_affine(value.target),
            "target_twist": None if target_twist is None else encode_twist(target_twist),
            "target_acceleration": (
                None if target_acceleration is None else encode_twist_acceleration(target_acceleration)
            ),
        }
    return value


def encode_cartesian_gain(value: Any | None) -> Any:
    if value is None:
        return None
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        value = tolist()
    rows = [list(row) if isinstance(row, (list, tuple)) else row for row in value]
    if len(rows) == 6 and all(isinstance(row, (list, tuple)) for row in rows):
        return [[float(item) for item in row] for row in rows]
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


def encode_twist_acceleration(value: Any) -> dict[str, Any]:
    return {
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
    params = getattr(motion, "params", motion)
    safety = getattr(params, "safety", params)
    cartesian_gains = getattr(params, "cartesian_gains", None)

    def field(name: str, default: Any = None) -> Any:
        return getattr(motion, name, getattr(params, name, default))

    def safety_field(name: str, default: Any = None) -> Any:
        return getattr(motion, name, getattr(safety, name, default))

    return {
        "target": encode_vector(motion.target),
        "target_velocity": encode_vector(motion.target_velocity),
        "stiffness": encode_vector(field("stiffness")),
        "damping": encode_vector(field("damping")),
        "error_clip": encode_vector(field("error_clip")),
        "constant_torque_offset": encode_vector(field("constant_torque_offset")),
        "lower_joint_limits": encode_vector(safety_field("lower_joint_limits")),
        "upper_joint_limits": encode_vector(safety_field("upper_joint_limits")),
        "compensate_coriolis": bool(field("compensate_coriolis")),
        "max_delta_tau": float(safety_field("max_delta_tau")),
        "joint_limit_activation_distance": float(safety_field("joint_limit_activation_distance")),
        "joint_limit_stiffness": float(safety_field("joint_limit_stiffness")),
        "joint_limit_damping": float(safety_field("joint_limit_damping")),
        "joint_limit_max_torque": float(safety_field("joint_limit_max_torque")),
        "friction": encode_friction_compensation_params(field("friction")),
        "cartesian_stiffness": encode_cartesian_diag(getattr(cartesian_gains, "stiffness", None)),
        "cartesian_damping": encode_cartesian_diag(getattr(cartesian_gains, "damping", None)),
    }


def encode_cartesian_impedance_fields(motion: Any) -> dict[str, Any]:
    params = getattr(motion, "params", motion)
    safety = getattr(params, "safety", params)

    def field(name: str, default: Any = None) -> Any:
        return getattr(motion, name, getattr(params, name, default))

    def safety_field(name: str, default: Any = None) -> Any:
        return getattr(motion, name, getattr(safety, name, default))

    nullspace_tasks = list(field("nullspace_tasks") or [])
    unsupported_tasks = [
        task for task in nullspace_tasks
        if type(task).__name__ not in {"PostureTask", "ManipulabilityTask"}
    ]
    if unsupported_tasks:
        raise ProtocolError("Unsupported Cartesian impedance nullspace task")
    posture_tasks = [task for task in nullspace_tasks if type(task).__name__ == "PostureTask"]
    manipulability_tasks = [task for task in nullspace_tasks if type(task).__name__ == "ManipulabilityTask"]
    if len(posture_tasks) > 1 or len(manipulability_tasks) > 1:
        raise ProtocolError("CartesianImpedanceMotion supports at most one nullspace task of each type")
    posture_task = posture_tasks[0] if posture_tasks else None
    manipulability_task = manipulability_tasks[0] if manipulability_tasks else None

    return {
        "target": encode_affine(motion.target),
        "target_twist": (
            encode_twist(motion.target_twist)
            if getattr(motion, "target_twist", None) is not None
            else None
        ),
        "target_type": encode_reference_type(field("target_type")),
        "stiffness": encode_matrix(field("stiffness")),
        "damping": encode_matrix(field("damping")),
        "translational_stiffness": (
            None if (v := field("translational_stiffness")) is None else float(v)
        ),
        "rotational_stiffness": None if (v := field("rotational_stiffness")) is None else float(v),
        "force_constraints": encode_optional_float_vector(field("force_constraints")),
        "posture_task": encode_rpc_value(posture_task),
        "manipulability_task": encode_rpc_value(manipulability_task),
        "max_delta_tau": float(safety_field("max_delta_tau")),
        "lower_joint_limits": encode_vector(safety_field("lower_joint_limits")),
        "upper_joint_limits": encode_vector(safety_field("upper_joint_limits")),
        "joint_limit_activation_distance": float(safety_field("joint_limit_activation_distance")),
        "joint_limit_stiffness": float(safety_field("joint_limit_stiffness")),
        "joint_limit_damping": float(safety_field("joint_limit_damping")),
        "joint_limit_max_torque": float(safety_field("joint_limit_max_torque")),
        "translational_error_clip": encode_vector(field("translational_error_clip")),
        "rotational_error_clip": encode_vector(field("rotational_error_clip")),
        "friction": encode_friction_compensation_params(field("friction")),
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
    }


@motion_encoder("TorqueStopMotion")
def encode_torque_stop_motion(motion: Any) -> dict[str, Any]:
    params = getattr(motion, "params", motion)

    def field(name: str, default: Any = None) -> Any:
        return getattr(motion, name, getattr(params, name, default))

    return {
        "type": "TorqueStopMotion",
        "damping": encode_vector(field("damping")),
        "ramp_duration": float(field("ramp_duration", 0.2)),
        "velocity_epsilon": float(field("velocity_epsilon", 0.02)),
        "max_duration": float(field("max_duration", 2.0)),
        "compensate_coriolis": bool(field("compensate_coriolis", True)),
        "max_delta_tau": float(field("max_delta_tau", 1.0)),
    }


def _safe_twist(robot_state: Any, attr: str) -> list[float]:
    """Read an Optional[Twist] robot_state attribute (e.g. O_dP_EE_est) as a flat
    [linear x/y/z, angular x/y/z] list. Twist is a structured object (.linear/.angular
    3-vectors), not iterable itself, and franky-computed estimates may be None."""
    try:
        twist = getattr(robot_state, attr)
        if twist is None:
            return []
        return [float(value) for value in twist.linear] + [float(value) for value in twist.angular]
    except Exception:
        return []


def encode_motion_reference(motion: Any) -> dict[str, Any] | None:
    """Encode the reference a tracking motion last picked up, or None if unset.

    Rides the state stream: the reference socket is CONFLATE, so this is how a
    client tells which update the controller took.
    """
    get_reference = getattr(motion, "get_reference", None)
    if callable(get_reference):
        reference = get_reference()
        if reference is None:
            return None
        if hasattr(reference, "q"):
            return {
                "type": "JointReference",
                "q": encode_vector(reference.q),
                "dq": encode_vector(reference.dq),
                "tau_ff": encode_vector(reference.tau_ff),
            }
        target_twist = getattr(reference, "target_twist", None)
        target_acceleration = getattr(reference, "target_acceleration", None)
        return {
            "type": "CartesianReference",
            "target": encode_affine(reference.target),
            "target_twist": None if target_twist is None else encode_twist(target_twist),
            "target_acceleration": (
                None if target_acceleration is None else encode_twist_acceleration(target_acceleration)
            ),
        }
    # SimpleTorqueMotion has no reference; its commanded torque is the analogue.
    get_torque = getattr(motion, "get_torque", None)
    if callable(get_torque):
        return {"type": "Torque", "tau": encode_vector(get_torque())}
    return None

#: All RobotState fields supported by the full state profile.
FULL_STATE_FIELDS = (
    "EE_T_K",
    "F_T_EE",
    "F_T_NE",
    "F_x_Cee",
    "F_x_Cload",
    "F_x_Ctotal",
    "I_ee",
    "I_load",
    "I_total",
    "K_F_ext_hat_K",
    "NE_T_EE",
    "O_F_ext_hat_K",
    "O_T_EE",
    "O_T_EE_c",
    "O_T_EE_d",
    "O_dP_EE_c",
    "O_dP_EE_d",
    "O_dP_EE_est",
    "O_ddP_EE_c",
    "O_ddP_EE_est",
    "O_ddP_O",
    "cartesian_collision",
    "cartesian_contact",
    "control_command_success_rate",
    "current_errors",
    "ddelbow_c",
    "ddelbow_est",
    "ddq_d",
    "ddq_est",
    "delbow_c",
    "delbow_est",
    "dq",
    "dq_d",
    "dq_est",
    "dtau_J",
    "dtheta",
    "elbow",
    "elbow_c",
    "elbow_d",
    "joint_collision",
    "joint_contact",
    "last_motion_errors",
    "m_ee",
    "m_load",
    "m_total",
    "q",
    "q_d",
    "q_est",
    "robot_mode",
    "tau_J",
    "tau_J_d",
    "tau_ext_hat_filtered",
    "theta",
    "time",
)

#: Fields emitted by the default fast state profile.
FAST_STATE_FIELDS = (
    "q",
    "dq",
    "O_T_EE",
    "O_F_ext_hat_K",
    "K_F_ext_hat_K",
    "tau_ext_hat_filtered",
    "O_dP_EE_est",
    "time_step",
    "rel_time",
    "abs_time",
    "control_signal",
)
_SUPPORTED_STATE_FIELDS = frozenset((*FULL_STATE_FIELDS, "time_step", "rel_time", "abs_time", "control_signal"))


def normalize_state_fields(fields: Sequence[str] | str | None) -> tuple[str, ...] | None:
    """Resolve the fast/full profiles or validate a custom field sequence."""
    if fields is None or fields == "fast":
        return None
    if fields == "full":
        return FULL_STATE_FIELDS
    if isinstance(fields, str):
        raise TypeError("state_fields must be 'fast', 'full', or a sequence of field names")
    if any(not isinstance(field, str) for field in fields):
        raise TypeError("state_fields must contain only strings")
    selected = tuple(dict.fromkeys(fields))
    unknown = sorted(set(selected) - _SUPPORTED_STATE_FIELDS)
    if unknown:
        raise ValueError(f"Unsupported state fields: {', '.join(unknown)}")
    return selected


def _encode_state_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, bytes, bool, int, float)):
        return value
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, dict):
        return {str(key): _encode_state_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_encode_state_value(item) for item in value]
    to_sec = getattr(value, "to_sec", None)
    if callable(to_sec):
        return float(to_sec())
    matrix = getattr(value, "matrix", None)
    if matrix is not None:
        return encode_matrix(matrix)
    linear = getattr(value, "linear", None)
    angular = getattr(value, "angular", None)
    if linear is not None and angular is not None:
        return [float(item) for item in linear] + [float(item) for item in angular]
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _encode_state_value(tolist())
    item = getattr(value, "item", None)
    if callable(item):
        return _encode_state_value(item())
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    properties = {}
    for field in dir(type(value)):
        if field.startswith("_") or not isinstance(getattr(type(value), field, None), property):
            continue
        try:
            properties[field] = _encode_state_value(getattr(value, field))
        except Exception:
            continue
    if properties:
        return properties
    try:
        return [_encode_state_value(item) for item in value]
    except TypeError:
        return repr(value)


def encode_callback_state(
    robot_state: Any,
    time_step: Any,
    rel_time: Any,
    abs_time: Any,
    control_signal: Any,
    fields: Sequence[str] | None = None,
) -> dict[str, Any]:
    if fields is None:
        return {
            "q": [float(value) for value in robot_state.q],
            "dq": [float(value) for value in robot_state.dq],
            "O_T_EE": [[float(value) for value in row] for row in robot_state.O_T_EE.matrix],
            "O_F_ext_hat_K": [float(value) for value in robot_state.O_F_ext_hat_K],
            "K_F_ext_hat_K": [float(value) for value in robot_state.K_F_ext_hat_K],
            "tau_ext_hat_filtered": [float(value) for value in robot_state.tau_ext_hat_filtered],
            "O_dP_EE_est": _safe_twist(robot_state, "O_dP_EE_est"),
            "time_step": float(time_step.to_sec()),
            "rel_time": float(rel_time.to_sec()),
            "abs_time": float(abs_time.to_sec()),
            "control_signal": _encode_control_signal(control_signal),
        }

    payload: dict[str, Any] = {}
    for field in fields:
        if field in ("time_step", "rel_time", "abs_time"):
            value = {"time_step": time_step, "rel_time": rel_time, "abs_time": abs_time}[field]
            payload[field] = float(value.to_sec())
        elif field == "control_signal":
            payload[field] = _encode_control_signal(control_signal)
        elif field == "O_dP_EE_est":
            payload[field] = _safe_twist(robot_state, field)
        else:
            payload[field] = _encode_state_value(getattr(robot_state, field))
    return payload


def _encode_control_signal(control_signal: Any):
    if hasattr(control_signal, "q"):
        return {"type": "JointPositions", "q": [float(value) for value in control_signal.q]}
    if hasattr(control_signal, "position"):
        return {"type": "JointPositions", "q": [float(value) for value in control_signal.position]}
    return {"type": type(control_signal).__name__, "repr": repr(control_signal)}
