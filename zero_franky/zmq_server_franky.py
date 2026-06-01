from __future__ import annotations

from typing import Any


MOTION_BUILDERS = {}


def motion_builder(type_name: str):
    def register(fn):
        MOTION_BUILDERS[type_name] = fn
        return fn

    return register


def _franky_reference_type(franky, value: str):
    return getattr(franky.ReferenceType, value)


def _franky_cartesian_impedance_dynamics_mode(franky, value: str | None):
    if value is None:
        return None
    return getattr(franky.CartesianImpedanceDynamicsMode, value)


def _franky_relative_dynamics_factor(franky, payload: dict[str, Any]):
    velocity = payload["velocity"]
    acceleration = payload["acceleration"]
    jerk = payload["jerk"]
    if velocity == acceleration == jerk:
        return franky.RelativeDynamicsFactor(velocity)
    return franky.RelativeDynamicsFactor(velocity, acceleration, jerk)


def _franky_duration(franky, value: int | None):
    if value is None:
        return None
    return franky.Duration(value)


def _franky_affine(franky, payload: dict[str, Any]):
    return franky.Affine(payload["matrix"])


def _kwargs_without_none(payload: dict[str, Any], *keys: str):
    return {key: payload[key] for key in keys if payload.get(key) is not None}


def _franky_nullspace_task(franky, payload: dict[str, Any]):
    type_name = payload["type"]
    if type_name == "PostureTask":
        return franky.PostureTask(
            payload["target"],
            payload["stiffness"],
            payload.get("damping"),
            payload["max_torque"],
        )
    if type_name == "ManipulabilityTask":
        return franky.ManipulabilityTask(
            payload["gain"],
            payload["damping"],
            payload["max_torque"],
            payload["finite_difference_step"],
        )
    raise ValueError(f"Unsupported nullspace task type: {type_name}")


def _franky_nullspace_tasks(franky, payload: list[dict[str, Any]] | None):
    if payload is None:
        return None
    return [_franky_nullspace_task(franky, item) for item in payload]


def franky_motion_kwargs(franky, kwargs: dict[str, Any] | None) -> dict[str, Any]:
    result = dict(kwargs or {})
    if "nullspace_tasks" in result:
        result["nullspace_tasks"] = _franky_nullspace_tasks(franky, result["nullspace_tasks"])
    if "dynamics_mode" in result:
        result["dynamics_mode"] = _franky_cartesian_impedance_dynamics_mode(franky, result["dynamics_mode"])
    return result


def _cartesian_impedance_kwargs(franky, payload: dict[str, Any]) -> dict[str, Any]:
    kwargs = {
        "target_type": _franky_reference_type(franky, payload["target_type"]),
        "translational_stiffness": payload["translational_stiffness"],
        "rotational_stiffness": payload["rotational_stiffness"],
        "force_constraints": payload["force_constraints"],
        "dynamics_mode": _franky_cartesian_impedance_dynamics_mode(franky, payload.get("dynamics_mode")),
        "nullspace_tasks": _franky_nullspace_tasks(franky, payload.get("nullspace_tasks")),
        "max_delta_tau": payload["max_delta_tau"],
        "lower_joint_limits": payload["lower_joint_limits"],
        "upper_joint_limits": payload["upper_joint_limits"],
        "joint_limit_activation_distance": payload["joint_limit_activation_distance"],
        "joint_limit_stiffness": payload["joint_limit_stiffness"],
        "joint_limit_damping": payload["joint_limit_damping"],
        "joint_limit_max_torque": payload["joint_limit_max_torque"],
        "translational_error_clip": payload["translational_error_clip"],
        "rotational_error_clip": payload["rotational_error_clip"],
    }
    return {key: value for key, value in kwargs.items() if value is not None}


def _franky_cartesian_target(franky, payload: dict[str, Any]):
    if payload["type"] == "Affine":
        return _franky_affine(franky, payload["value"])
    if payload["type"] == "RobotPose":
        return franky.RobotPose(_franky_affine(franky, payload["end_effector_pose"]))
    if payload["type"] == "CartesianState":
        pose = franky.RobotPose(_franky_affine(franky, payload["pose"]["end_effector_pose"]))
        return franky.CartesianState(pose)
    raise ValueError(f"Unsupported cartesian target type: {payload['type']}")


def _franky_twist(franky, payload: dict[str, Any]):
    return franky.Twist(payload["linear"], payload["angular"])


def _franky_robot_velocity(franky, payload: dict[str, Any]):
    if payload["type"] == "RobotVelocity":
        return franky.RobotVelocity(_franky_twist(franky, payload["end_effector_twist"]), payload["elbow_velocity"])
    if payload["type"] == "Twist":
        return _franky_twist(franky, payload["value"])
    raise ValueError(f"Unsupported robot velocity type: {payload['type']}")


def _franky_position_waypoint(franky, waypoint: dict[str, Any], cls, target_builder):
    return cls(
        target_builder(franky, waypoint["target"]),
        _franky_reference_type(franky, waypoint["reference_type"]),
        _franky_relative_dynamics_factor(franky, waypoint["relative_dynamics_factor"]),
        _franky_duration(franky, waypoint["minimum_time"]),
        _franky_duration(franky, waypoint["hold_target_duration"]),
        _franky_duration(franky, waypoint["max_total_duration"]),
    )


def _franky_velocity_waypoint(franky, waypoint: dict[str, Any], cls, target_builder):
    return cls(
        target_builder(franky, waypoint["target"]),
        _franky_relative_dynamics_factor(franky, waypoint["relative_dynamics_factor"]),
        _franky_duration(franky, waypoint["minimum_time"]),
        _franky_duration(franky, waypoint["hold_target_duration"]),
        _franky_duration(franky, waypoint["max_total_duration"]),
    )


def _franky_joint_target(franky, payload: dict[str, Any]):
    return franky.JointState(payload["position"])


def _identity_target_builder(_, payload):
    return payload


def build_franky_motion(franky, payload: dict[str, Any]):
    type_name = payload["type"]
    try:
        return MOTION_BUILDERS[type_name](franky, payload)
    except KeyError as exc:
        raise ValueError(f"Unsupported motion type: {type_name}") from exc


@motion_builder("CartesianMotion")
def build_cartesian_motion(franky, payload: dict[str, Any]):
    ee_frame = payload.get("ee_frame")
    return franky.CartesianMotion(
        _franky_cartesian_target(franky, payload["target"]),
        _franky_reference_type(franky, payload["reference_type"]),
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
        payload["return_when_finished"],
        _franky_affine(franky, ee_frame) if ee_frame is not None else None,
    )


@motion_builder("CartesianWaypointMotion")
def build_cartesian_waypoint_motion(franky, payload: dict[str, Any]):
    ee_frame = payload.get("ee_frame")
    return franky.CartesianWaypointMotion(
        [
            _franky_position_waypoint(franky, waypoint, franky.CartesianWaypoint, _franky_cartesian_target)
            for waypoint in payload["waypoints"]
        ],
        _franky_affine(franky, ee_frame) if ee_frame is not None else None,
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
        payload["return_when_finished"],
    )


@motion_builder("CartesianStopMotion")
def build_cartesian_stop_motion(franky, payload: dict[str, Any]):
    return franky.CartesianStopMotion(_franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]))


@motion_builder("JointMotion")
def build_joint_motion(franky, payload: dict[str, Any]):
    return franky.JointMotion(
        franky.JointState(payload["target"]["position"]),
        _franky_reference_type(franky, payload["reference_type"]),
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
        payload["return_when_finished"],
    )


@motion_builder("JointWaypointMotion")
def build_joint_waypoint_motion(franky, payload: dict[str, Any]):
    return franky.JointWaypointMotion(
        [
            _franky_position_waypoint(franky, waypoint, franky.JointWaypoint, _franky_joint_target)
            for waypoint in payload["waypoints"]
        ],
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
        payload["return_when_finished"],
    )


@motion_builder("JointStopMotion")
def build_joint_stop_motion(franky, payload: dict[str, Any]):
    return franky.JointStopMotion(_franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]))


@motion_builder("CartesianVelocityMotion")
def build_cartesian_velocity_motion(franky, payload: dict[str, Any]):
    ee_frame = payload.get("ee_frame")
    return franky.CartesianVelocityMotion(
        _franky_robot_velocity(franky, payload["target"]),
        _franky_duration(franky, payload["duration"]),
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
        _franky_affine(franky, ee_frame) if ee_frame is not None else None,
    )


@motion_builder("CartesianVelocityWaypointMotion")
def build_cartesian_velocity_waypoint_motion(franky, payload: dict[str, Any]):
    ee_frame = payload.get("ee_frame")
    return franky.CartesianVelocityWaypointMotion(
        [
            _franky_velocity_waypoint(franky, waypoint, franky.CartesianVelocityWaypoint, _franky_robot_velocity)
            for waypoint in payload["waypoints"]
        ],
        _franky_affine(franky, ee_frame) if ee_frame is not None else None,
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
    )


@motion_builder("CartesianVelocityStopMotion")
def build_cartesian_velocity_stop_motion(franky, payload: dict[str, Any]):
    return franky.CartesianVelocityStopMotion(
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"])
    )


@motion_builder("JointVelocityMotion")
def build_joint_velocity_motion(franky, payload: dict[str, Any]):
    return franky.JointVelocityMotion(
        payload["target"],
        _franky_duration(franky, payload["duration"]),
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
    )


@motion_builder("JointVelocityWaypointMotion")
def build_joint_velocity_waypoint_motion(franky, payload: dict[str, Any]):
    return franky.JointVelocityWaypointMotion(
        [
            _franky_velocity_waypoint(franky, waypoint, franky.JointVelocityWaypoint, _identity_target_builder)
            for waypoint in payload["waypoints"]
        ],
        _franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]),
    )


@motion_builder("JointVelocityStopMotion")
def build_joint_velocity_stop_motion(franky, payload: dict[str, Any]):
    return franky.JointVelocityStopMotion(_franky_relative_dynamics_factor(franky, payload["relative_dynamics_factor"]))


@motion_builder("JointImpedanceMotion")
def build_joint_impedance_motion(franky, payload: dict[str, Any]):
    kwargs = _kwargs_without_none(
        payload,
        "target_velocity",
        "stiffness",
        "damping",
        "constant_torque_offset",
        "lower_joint_limits",
        "upper_joint_limits",
        "friction_coulomb",
        "friction_viscous",
        "friction_max_torque",
    )
    kwargs.update(
        {
            "compensate_coriolis": payload["compensate_coriolis"],
            "max_delta_tau": payload["max_delta_tau"],
            "joint_limit_activation_distance": payload["joint_limit_activation_distance"],
            "joint_limit_stiffness": payload["joint_limit_stiffness"],
            "joint_limit_damping": payload["joint_limit_damping"],
            "joint_limit_max_torque": payload["joint_limit_max_torque"],
            "friction_velocity_epsilon": payload.get("friction_velocity_epsilon", 0.03),
        }
    )
    return franky.JointImpedanceMotion(payload["target"], **kwargs)


@motion_builder("CartesianImpedanceMotion")
def build_cartesian_impedance_motion(franky, payload: dict[str, Any]):
    kwargs = _cartesian_impedance_kwargs(franky, payload)
    kwargs.update(
        {
            "return_when_finished": payload["return_when_finished"],
            "finish_wait_factor": payload["finish_wait_factor"],
        }
    )
    return franky.CartesianImpedanceMotion(
        _franky_affine(franky, payload["target"]),
        _franky_duration(franky, payload["duration"]),
        **kwargs,
    )


@motion_builder("ExponentialImpedanceMotion")
def build_exponential_impedance_motion(franky, payload: dict[str, Any]):
    kwargs = _cartesian_impedance_kwargs(franky, payload)
    kwargs["exponential_decay"] = payload["exponential_decay"]
    return franky.ExponentialImpedanceMotion(_franky_affine(franky, payload["target"]), **kwargs)
