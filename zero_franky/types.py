"""Client-side stand-ins for the `franky` types that `Robot.move` accepts.

`zero_franky`'s encoders never import `franky`; they read attributes off whatever
they are handed and dispatch on `type(motion).__name__` (see
`zero_franky.protocol.encode_motion`). So a client only needs objects whose class
names and attributes match franky's. This module provides them, letting an
application host build motions with no `franky-control` wheel and no libfranka.

Constructor names, argument order, and defaults mirror franky's pybind11
bindings, so code written against either one ports unchanged:

    from zero_franky.types import Affine, CartesianMotion, ReferenceType

    motion = CartesianMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)

Real `franky` objects remain fully supported — on a host that has franky, pass
those instead and the same encoders handle them. The two produce identical wire
payloads, which `tests/test_types_parity.py` asserts wherever franky is
importable.

"""

from __future__ import annotations

import math
from enum import Enum
from typing import Any, Iterable, Sequence

__all__ = [
    "CRITICAL",
    "Affine",
    "CartesianImpedanceGains",
    "CartesianMotion",
    "CartesianReference",
    "CartesianState",
    "CartesianStopMotion",
    "CartesianVelocityMotion",
    "CartesianVelocityStopMotion",
    "CartesianVelocityWaypoint",
    "CartesianVelocityWaypointMotion",
    "CartesianWaypoint",
    "CartesianWaypointMotion",
    "Duration",
    "FrictionCompensationParams",
    "JointImpedanceGains",
    "JointMotion",
    "JointReference",
    "JointState",
    "JointStopMotion",
    "JointVelocityMotion",
    "JointVelocityStopMotion",
    "JointVelocityWaypoint",
    "JointVelocityWaypointMotion",
    "JointWaypoint",
    "JointWaypointMotion",
    "ManipulabilityTask",
    "NullspaceGains",
    "PostureTask",
    "ReferenceType",
    "RelativeDynamicsFactor",
    "RobotPose",
    "RobotVelocity",
    "Twist",
    "TwistAcceleration",
]


# --- matrix helpers ---------------------------------------------------------
#
# Deliberately dependency-free: numpy is not a client requirement, and these
# only ever run on 3- and 4-vectors and 4x4 matrices.


def _vector(value: Iterable[Any], length: int, name: str) -> tuple[float, ...]:
    items = tuple(float(item) for item in value)
    if len(items) != length:
        raise ValueError(f"{name} must have {length} elements, got {len(items)}")
    return items


def _looks_like_matrix4(value: Any) -> bool:
    try:
        rows = list(value)
    except TypeError:
        return False
    return len(rows) == 4 and all(hasattr(row, "__len__") and len(row) == 4 for row in rows)


def _rotation_from_quaternion(quaternion: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    """Build a rotation matrix from an [x, y, z, w] quaternion.

    Transcribes Eigen's `Quaternion::toRotationMatrix`, including its behaviour
    on non-unit input (the result is scaled by the squared norm rather than
    normalized), so payloads match what franky would have produced.
    """
    x, y, z, w = _vector(quaternion, 4, "quaternion")
    tx, ty, tz = 2.0 * x, 2.0 * y, 2.0 * z
    twx, twy, twz = tx * w, ty * w, tz * w
    txx, txy, txz = tx * x, ty * x, tz * x
    tyy, tyz, tzz = ty * y, tz * y, tz * z
    return (
        (1.0 - (tyy + tzz), txy - twz, txz + twy),
        (txy + twz, 1.0 - (txx + tzz), tyz - twx),
        (txz - twy, tyz + twx, 1.0 - (txx + tyy)),
    )


def _quaternion_from_rotation(rotation: Sequence[Sequence[float]]) -> tuple[float, ...]:
    """Recover an [x, y, z, w] quaternion from a rotation matrix.

    Branches on the largest diagonal term to stay numerically stable near the
    half-turn cases where the naive trace formula loses precision.
    """
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = (tuple(row) for row in rotation)
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale)
    if m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale, (m21 - m12) / scale)
    if m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale, (m02 - m20) / scale)
    scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale, (m10 - m01) / scale)


def _matmul4(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]):
    return tuple(
        tuple(sum(left[row][k] * right[k][col] for k in range(4)) for col in range(4))
        for row in range(4)
    )


class Affine:
    """A rigid 3D transformation: a translation plus a rotation.

    Accepts either of franky's two constructor forms — a 4x4 homogeneous matrix,
    or a translation [m] with an [x, y, z, w] quaternion:

        Affine([0.2, 0.0, 0.0])                    # translation only
        Affine([0.2, 0.0, 0.0], [0, 0, 0, 1])      # translation + rotation
        Affine(transformation_matrix=matrix)        # 4x4

    A single positional argument is read as a matrix when it is 4x4 and as a
    translation when it has three elements.
    """

    def __init__(
        self,
        translation_or_matrix: Any = None,
        quaternion: Sequence[float] | None = None,
        *,
        translation: Sequence[float] | None = None,
        transformation_matrix: Sequence[Sequence[float]] | None = None,
    ):
        if translation_or_matrix is not None:
            if _looks_like_matrix4(translation_or_matrix):
                if transformation_matrix is not None:
                    raise TypeError("Affine got a matrix both positionally and as transformation_matrix")
                transformation_matrix = translation_or_matrix
            else:
                if translation is not None:
                    raise TypeError("Affine got a translation both positionally and as translation")
                translation = translation_or_matrix

        if transformation_matrix is not None:
            if translation is not None or quaternion is not None:
                raise TypeError("Affine takes either transformation_matrix or translation/quaternion, not both")
            self._matrix = tuple(_vector(row, 4, "matrix row") for row in transformation_matrix)
            return

        offset = _vector(translation, 3, "translation") if translation is not None else (0.0, 0.0, 0.0)
        rotation = _rotation_from_quaternion(quaternion if quaternion is not None else (0.0, 0.0, 0.0, 1.0))
        self._matrix = (
            rotation[0] + (offset[0],),
            rotation[1] + (offset[1],),
            rotation[2] + (offset[2],),
            (0.0, 0.0, 0.0, 1.0),
        )

    @property
    def matrix(self):
        """This transformation as a 4x4 homogeneous matrix."""
        return self._matrix

    @property
    def translation(self):
        """The translation component [m]."""
        return tuple(row[3] for row in self._matrix[:3])

    @property
    def quaternion(self):
        """The rotation component as an [x, y, z, w] quaternion.

        Reads the rotation block as-is, so it assumes that block is orthonormal.
        Given a non-unit quaternion, franky's accessor recovers the orthonormal
        factor by polar decomposition and this one does not; `matrix` — all the
        wire carries — agrees either way.
        """
        return _quaternion_from_rotation([row[:3] for row in self._matrix[:3]])

    @property
    def inverse(self) -> "Affine":
        """The inverse transformation.

        Computed as the rigid inverse (transposed rotation, negated translation),
        which assumes the rotation block is orthonormal — true for anything built
        from a translation and a unit quaternion. franky takes a general inverse
        instead, so the two diverge only for a non-unit quaternion.
        """
        rotation = [row[:3] for row in self._matrix[:3]]
        offset = self.translation
        transposed = [[rotation[col][row] for col in range(3)] for row in range(3)]
        return Affine(
            transformation_matrix=[
                list(transposed[row]) + [-sum(transposed[row][k] * offset[k] for k in range(3))]
                for row in range(3)
            ]
            + [[0.0, 0.0, 0.0, 1.0]]
        )

    def __mul__(self, other: "Affine") -> "Affine":
        if not isinstance(other, Affine):
            return NotImplemented
        return Affine(transformation_matrix=_matmul4(self._matrix, other.matrix))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Affine):
            return NotImplemented
        return self._matrix == other._matrix

    def __repr__(self) -> str:
        translation = ", ".join(f"{item:.4g}" for item in self.translation)
        quaternion = ", ".join(f"{item:.4g}" for item in self.quaternion)
        return f"Affine(translation=[{translation}], quaternion=[{quaternion}])"


class Duration:
    """A duration in milliseconds, matching `franka::Duration`."""

    def __init__(self, milliseconds: int = 0):
        self._milliseconds = int(milliseconds)

    def to_msec(self) -> int:
        return self._milliseconds

    def to_sec(self) -> float:
        return self._milliseconds / 1000.0

    def __add__(self, other: "Duration") -> "Duration":
        if not isinstance(other, Duration):
            return NotImplemented
        return Duration(self._milliseconds + other._milliseconds)

    def __sub__(self, other: "Duration") -> "Duration":
        if not isinstance(other, Duration):
            return NotImplemented
        return Duration(self._milliseconds - other._milliseconds)

    def __mul__(self, factor: int) -> "Duration":
        return Duration(self._milliseconds * int(factor))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Duration):
            return NotImplemented
        return self._milliseconds == other._milliseconds

    def __hash__(self) -> int:
        return hash(self._milliseconds)

    def __repr__(self) -> str:
        return f"Duration({self._milliseconds})"


class ReferenceType(Enum):
    """Whether a motion target is absolute or relative to the current pose.

    `zero_franky.protocol.encode_reference_type` matches on the member name, so
    these members encode the same as franky's enum. Note that a bare string is
    *not* accepted in its place.
    """

    Relative = "Relative"
    Absolute = "Absolute"


class RelativeDynamicsFactor:
    """Per-derivative scaling of a motion's velocity, acceleration, and jerk.

    A plain float works anywhere this type does — the encoder broadcasts it to
    all three components — so reach for this only to scale them separately.
    """

    def __init__(self, velocity: float = 1.0, acceleration: float | None = None, jerk: float | None = None):
        self.velocity = float(velocity)
        self.acceleration = self.velocity if acceleration is None else float(acceleration)
        self.jerk = self.velocity if jerk is None else float(jerk)

    def __repr__(self) -> str:
        return (
            f"RelativeDynamicsFactor(velocity={self.velocity:.4g}, "
            f"acceleration={self.acceleration:.4g}, jerk={self.jerk:.4g})"
        )


class Twist:
    """A spatial velocity: linear [m/s] and angular [rad/s] components.

    Note franky's asymmetry, reproduced here: the constructor arguments are
    `linear_velocity`/`angular_velocity` while the properties are
    `linear`/`angular`.
    """

    def __init__(
        self,
        linear_velocity: Sequence[float] | None = None,
        angular_velocity: Sequence[float] | None = None,
    ):
        self.linear = _vector(linear_velocity, 3, "linear_velocity") if linear_velocity is not None else (0.0,) * 3
        self.angular = _vector(angular_velocity, 3, "angular_velocity") if angular_velocity is not None else (0.0,) * 3

    def __repr__(self) -> str:
        return f"Twist(linear_velocity={list(self.linear)}, angular_velocity={list(self.angular)})"


class TwistAcceleration:
    """A spatial acceleration, with the same layout as `Twist`."""

    def __init__(
        self,
        linear_acceleration: Sequence[float] | None = None,
        angular_acceleration: Sequence[float] | None = None,
    ):
        self.linear = (
            _vector(linear_acceleration, 3, "linear_acceleration") if linear_acceleration is not None else (0.0,) * 3
        )
        self.angular = (
            _vector(angular_acceleration, 3, "angular_acceleration") if angular_acceleration is not None else (0.0,) * 3
        )

    def __repr__(self) -> str:
        return f"TwistAcceleration(linear_acceleration={list(self.linear)}, angular_acceleration={list(self.angular)})"


class RobotVelocity:
    """An end-effector twist, optionally with an elbow velocity [rad/s]."""

    def __init__(self, end_effector_twist: Twist, elbow_velocity: float | None = None):
        self.end_effector_twist = end_effector_twist
        self.elbow_velocity = None if elbow_velocity is None else float(elbow_velocity)

    def __repr__(self) -> str:
        return f"RobotVelocity({self.end_effector_twist!r}, elbow_velocity={self.elbow_velocity})"


class RobotPose:
    """An end-effector pose."""

    def __init__(self, end_effector_pose: Affine):
        self.end_effector_pose = end_effector_pose

    def __repr__(self) -> str:
        return f"RobotPose({self.end_effector_pose!r})"


class CartesianState:
    """A Cartesian target, wrapping a `RobotPose`."""

    def __init__(self, pose: RobotPose):
        self.pose = pose

    def __repr__(self) -> str:
        return f"CartesianState({self.pose!r})"


class JointState:
    """A joint-space target: seven joint positions [rad] and optional velocities [rad/s]."""

    def __init__(self, position: Sequence[float], velocity: Sequence[float] | None = None):
        self.position = _vector(position, 7, "position")
        self.velocity = (0.0,) * 7 if velocity is None else _vector(velocity, 7, "velocity")

    def __repr__(self) -> str:
        return f"JointState({list(self.position)}, {list(self.velocity)})"


def _as_cartesian_state(target: Any) -> CartesianState:
    """Normalize a Cartesian target to a `CartesianState`.

    franky's motions do this internally, so their `.target` always reads back as
    a `CartesianState`. Matching it keeps the encoded payload identical whichever
    of the three accepted forms the caller passed.
    """
    if isinstance(target, CartesianState):
        return target
    if isinstance(target, RobotPose):
        return CartesianState(target)
    if isinstance(target, Affine):
        return CartesianState(RobotPose(target))
    if getattr(target, "matrix", None) is not None:
        return CartesianState(RobotPose(target))
    if hasattr(target, "pose"):
        return target
    if hasattr(target, "end_effector_pose"):
        return CartesianState(target)
    raise TypeError(f"Cannot use {type(target).__name__} as a Cartesian target")


def _as_joint_state(target: Any) -> JointState:
    if isinstance(target, JointState):
        return target
    position = getattr(target, "position", target)
    return JointState(position)


class CartesianWaypoint:
    """One waypoint of a Cartesian position motion."""

    def __init__(
        self,
        target: Any,
        reference_type: ReferenceType = ReferenceType.Absolute,
        relative_dynamics_factor: Any = 1.0,
        minimum_time: Duration | None = None,
        hold_target_duration: Duration | None = None,
        max_total_duration: Duration | None = None,
    ):
        self.target = _as_cartesian_state(target)
        self.reference_type = reference_type
        self.relative_dynamics_factor = relative_dynamics_factor
        self.minimum_time = minimum_time
        self.hold_target_duration = Duration(0) if hold_target_duration is None else hold_target_duration
        self.max_total_duration = max_total_duration


class JointWaypoint:
    """One waypoint of a joint position motion."""

    def __init__(
        self,
        target: Any,
        reference_type: ReferenceType = ReferenceType.Absolute,
        relative_dynamics_factor: Any = 1.0,
        minimum_time: Duration | None = None,
        hold_target_duration: Duration | None = None,
        max_total_duration: Duration | None = None,
    ):
        self.target = _as_joint_state(target)
        self.reference_type = reference_type
        self.relative_dynamics_factor = relative_dynamics_factor
        self.minimum_time = minimum_time
        self.hold_target_duration = Duration(0) if hold_target_duration is None else hold_target_duration
        self.max_total_duration = max_total_duration


class CartesianVelocityWaypoint:
    """One waypoint of a Cartesian velocity motion.

    Velocity waypoints carry no reference type — a velocity target is always
    absolute.
    """

    def __init__(
        self,
        target: RobotVelocity | Twist,
        relative_dynamics_factor: Any = 1.0,
        minimum_time: Duration | None = None,
        hold_target_duration: Duration | None = None,
        max_total_duration: Duration | None = None,
    ):
        self.target = target
        self.relative_dynamics_factor = relative_dynamics_factor
        self.minimum_time = minimum_time
        self.hold_target_duration = Duration(0) if hold_target_duration is None else hold_target_duration
        self.max_total_duration = max_total_duration


class JointVelocityWaypoint:
    """One waypoint of a joint velocity motion."""

    def __init__(
        self,
        target: Sequence[float],
        relative_dynamics_factor: Any = 1.0,
        minimum_time: Duration | None = None,
        hold_target_duration: Duration | None = None,
        max_total_duration: Duration | None = None,
    ):
        self.target = _vector(target, 7, "target")
        self.relative_dynamics_factor = relative_dynamics_factor
        self.minimum_time = minimum_time
        self.hold_target_duration = Duration(0) if hold_target_duration is None else hold_target_duration
        self.max_total_duration = max_total_duration


class CartesianWaypointMotion:
    """A Cartesian position motion through a sequence of waypoints."""

    def __init__(
        self,
        waypoints: Sequence[CartesianWaypoint],
        ee_frame: Affine | None = None,
        relative_dynamics_factor: Any = 1.0,
        return_when_finished: bool = True,
    ):
        self.waypoints = list(waypoints)
        self.ee_frame = ee_frame
        self.relative_dynamics_factor = relative_dynamics_factor
        self.return_when_finished = return_when_finished


class CartesianMotion(CartesianWaypointMotion):
    """A Cartesian position motion to a single target.

    The target may be an `Affine`, a `RobotPose`, or a `CartesianState`.
    """

    def __init__(
        self,
        target: Any,
        reference_type: ReferenceType = ReferenceType.Absolute,
        relative_dynamics_factor: Any = 1.0,
        return_when_finished: bool = True,
        ee_frame: Affine | None = None,
    ):
        super().__init__(
            [CartesianWaypoint(target, reference_type, relative_dynamics_factor)],
            ee_frame=ee_frame,
            relative_dynamics_factor=relative_dynamics_factor,
            return_when_finished=return_when_finished,
        )
        self.target = _as_cartesian_state(target)
        self.reference_type = reference_type


class CartesianStopMotion:
    """Bring Cartesian pose control to a stop."""

    def __init__(self, relative_dynamics_factor: Any = 1.0):
        self.relative_dynamics_factor = relative_dynamics_factor


class JointWaypointMotion:
    """A joint position motion through a sequence of waypoints."""

    def __init__(
        self,
        waypoints: Sequence[JointWaypoint],
        relative_dynamics_factor: Any = 1.0,
        return_when_finished: bool = True,
    ):
        self.waypoints = list(waypoints)
        self.relative_dynamics_factor = relative_dynamics_factor
        self.return_when_finished = return_when_finished


class JointMotion(JointWaypointMotion):
    """A joint position motion to a single seven-element target [rad]."""

    def __init__(
        self,
        target: Sequence[float],
        reference_type: ReferenceType = ReferenceType.Absolute,
        relative_dynamics_factor: Any = 1.0,
        return_when_finished: bool = True,
    ):
        super().__init__(
            [JointWaypoint(target, reference_type, relative_dynamics_factor)],
            relative_dynamics_factor=relative_dynamics_factor,
            return_when_finished=return_when_finished,
        )
        self.target = _as_joint_state(target)
        self.reference_type = reference_type


class JointStopMotion:
    """Bring joint position control to a stop."""

    def __init__(self, relative_dynamics_factor: Any = 1.0):
        self.relative_dynamics_factor = relative_dynamics_factor


class CartesianVelocityWaypointMotion:
    """A Cartesian velocity motion through a sequence of waypoints."""

    def __init__(
        self,
        waypoints: Sequence[CartesianVelocityWaypoint],
        ee_frame: Affine | None = None,
        relative_dynamics_factor: Any = 1.0,
    ):
        self.waypoints = list(waypoints)
        self.ee_frame = ee_frame
        self.relative_dynamics_factor = relative_dynamics_factor


class CartesianVelocityMotion(CartesianVelocityWaypointMotion):
    """Hold a Cartesian velocity for `duration`.

    The target may be a `RobotVelocity` or a bare `Twist`.
    """

    def __init__(
        self,
        target: RobotVelocity | Twist,
        duration: Duration | None = None,
        relative_dynamics_factor: Any = 1.0,
        ee_frame: Affine | None = None,
    ):
        resolved = Duration(1000) if duration is None else duration
        super().__init__(
            [CartesianVelocityWaypoint(target, relative_dynamics_factor, hold_target_duration=resolved)],
            ee_frame=ee_frame,
            relative_dynamics_factor=relative_dynamics_factor,
        )
        self.target = target
        self.duration = resolved


class CartesianVelocityStopMotion:
    """Bring Cartesian velocity control to a stop."""

    def __init__(self, relative_dynamics_factor: Any = 1.0):
        self.relative_dynamics_factor = relative_dynamics_factor


class JointVelocityWaypointMotion:
    """A joint velocity motion through a sequence of waypoints."""

    def __init__(
        self,
        waypoints: Sequence[JointVelocityWaypoint],
        relative_dynamics_factor: Any = 1.0,
    ):
        self.waypoints = list(waypoints)
        self.relative_dynamics_factor = relative_dynamics_factor


class JointVelocityMotion(JointVelocityWaypointMotion):
    """Hold a joint velocity [rad/s] for `duration`."""

    def __init__(
        self,
        target: Sequence[float],
        duration: Duration | None = None,
        relative_dynamics_factor: Any = 1.0,
    ):
        resolved = Duration(1000) if duration is None else duration
        super().__init__(
            [JointVelocityWaypoint(target, relative_dynamics_factor, hold_target_duration=resolved)],
            relative_dynamics_factor=relative_dynamics_factor,
        )
        self.target = _vector(target, 7, "target")
        self.duration = resolved


class JointVelocityStopMotion:
    """Bring joint velocity control to a stop."""

    def __init__(self, relative_dynamics_factor: Any = 1.0):
        self.relative_dynamics_factor = relative_dynamics_factor


class _CriticalDamping:
    """Sentinel type for :data:`CRITICAL`."""

    __slots__ = ()

    def __repr__(self) -> str:
        return "franky.CRITICAL"


CRITICAL = _CriticalDamping()


class PostureTask:
    """Joint-posture objective projected into the Cartesian nullspace."""

    def __init__(
        self,
        target: Sequence[float],
        stiffness: float | Sequence[float],
        damping: float | Sequence[float] | None = None,
        max_torque: float | None = None,
    ):
        self.target = _vector(target, 7, "target")
        if isinstance(stiffness, (int, float)):
            self.stiffness: tuple[float, ...] = (float(stiffness),) * 7
        else:
            self.stiffness = _vector(stiffness, 7, "stiffness")
        if damping is None:
            self.damping: tuple[float, ...] | None = None
        elif isinstance(damping, (int, float)):
            self.damping = (float(damping),) * 7
        else:
            self.damping = _vector(damping, 7, "damping")
        self.max_torque: float | None = None if max_torque is None else float(max_torque)

    def __repr__(self) -> str:
        return (
            f"PostureTask(target={list(self.target)}, stiffness={list(self.stiffness)}, "
            f"damping={list(self.damping) if self.damping is not None else None}, "
            f"max_torque={self.max_torque})"
        )


class ManipulabilityTask:
    """Manipulability maximization objective projected into the Cartesian nullspace."""

    def __init__(
        self,
        gain: float,
        damping: float = 0.0,
        max_torque: float | None = None,
    ):
        self.gain = float(gain)
        self.damping = float(damping)
        self.max_torque = None if max_torque is None else float(max_torque)

    def __repr__(self) -> str:
        return f"ManipulabilityTask(gain={self.gain}, damping={self.damping}, max_torque={self.max_torque})"


class NullspaceGains:
    """Runtime-adjustable gains for a nullspace task."""

    def __init__(self):
        # franky's binding is `py::init<>()`: the gains are default-constructed
        # and then assigned. Taking constructor keywords here would let client
        # code compile against the stand-in and fail against franky.
        self._posture_stiffness: tuple[float, ...] = (0.0,) * 7
        self._posture_damping: tuple[float, ...] | None = None
        self.posture_max_torque: float | None = None
        self.manipulability_gain = 0.0
        self.manipulability_damping = 0.0
        self.manipulability_max_torque: float | None = None

    @property
    def posture_stiffness(self) -> tuple[float, ...]:
        return self._posture_stiffness

    @posture_stiffness.setter
    def posture_stiffness(self, value: float | Sequence[float]):
        if isinstance(value, (int, float)):
            self._posture_stiffness = (float(value),) * 7
        else:
            self._posture_stiffness = _vector(value, 7, "posture_stiffness")

    @property
    def posture_damping(self) -> tuple[float, ...] | None:
        return self._posture_damping

    @posture_damping.setter
    def posture_damping(self, value: float | Sequence[float] | None):
        if value is None:
            self._posture_damping = None
        elif isinstance(value, (int, float)):
            self._posture_damping = (float(value),) * 7
        else:
            self._posture_damping = _vector(value, 7, "posture_damping")

    def __repr__(self) -> str:
        return (
            f"NullspaceGains(posture_stiffness={list(self.posture_stiffness)}, "
            f"posture_damping={list(self.posture_damping) if self.posture_damping is not None else None}, "
            f"posture_max_torque={self.posture_max_torque}, "
            f"manipulability_gain={self.manipulability_gain}, "
            f"manipulability_damping={self.manipulability_damping}, "
            f"manipulability_max_torque={self.manipulability_max_torque})"
        )


class CartesianImpedanceGains:
    """Runtime-adjustable stiffness and damping gains for Cartesian impedance motions."""

    def __init__(
        self,
        translational_stiffness: float = 500.0,
        rotational_stiffness: float = 50.0,
    ):
        kt = float(translational_stiffness)
        kr = float(rotational_stiffness)
        self.stiffness: list[list[float]] = [
            [kt, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, kt, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, kt, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, kr, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, kr, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, kr],
        ]
        self.damping: list[list[float]] | None = None

    @classmethod
    def isotropic(
        cls,
        translational_stiffness: float,
        rotational_stiffness: float,
        translational_damping: float | None = None,
        rotational_damping: float | None = None,
    ) -> "CartesianImpedanceGains":
        kt = float(translational_stiffness)
        kr = float(rotational_stiffness)
        inst = cls(kt, kr)
        # Passing one damping component leaves the other critically damped, as
        # franky's `value_or(2 * sqrt(stiffness))` does. Filling it with zero
        # instead would silently leave a stiff axis undamped.
        if translational_damping is not None or rotational_damping is not None:
            dt = 2.0 * math.sqrt(kt) if translational_damping is None else float(translational_damping)
            dr = 2.0 * math.sqrt(kr) if rotational_damping is None else float(rotational_damping)
            inst.damping = [
                [dt, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, dt, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, dt, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, dr, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, dr, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, dr],
            ]
        return inst

    @classmethod
    def diagonal(
        cls,
        stiffness: Sequence[float],
        damping: Sequence[float] | None = None,
    ) -> "CartesianImpedanceGains":
        k = _vector(stiffness, 6, "stiffness")
        inst = cls(500.0, 50.0)
        inst.stiffness = [
            [k[i] if i == j else 0.0 for j in range(6)]
            for i in range(6)
        ]
        if damping is not None:
            d = _vector(damping, 6, "damping")
            inst.damping = [
                [d[i] if i == j else 0.0 for j in range(6)]
                for i in range(6)
            ]
        return inst

    def __repr__(self) -> str:
        return f"CartesianImpedanceGains(stiffness={self.stiffness}, damping={self.damping})"


class JointImpedanceGains:
    """Runtime-adjustable stiffness and damping gains for joint impedance motions."""

    def __init__(
        self,
        stiffness: Sequence[float] | None = None,
        damping: Sequence[float] | None = None,
    ):
        self.stiffness: tuple[float, ...] = (
            (50.0,) * 7 if stiffness is None else _vector(stiffness, 7, "stiffness")
        )
        self.damping: tuple[float, ...] | None = (
            None if damping is None else _vector(damping, 7, "damping")
        )

    def __repr__(self) -> str:
        return (
            f"JointImpedanceGains(stiffness={list(self.stiffness)}, "
            f"damping={list(self.damping) if self.damping is not None else None})"
        )


class FrictionCompensationParams:
    """Per-joint friction feedforward settings for torque-control motions."""

    def __init__(
        self,
        coulomb: Sequence[float] = (0.0,) * 7,
        viscous: Sequence[float] = (0.0,) * 7,
        max_torque: Sequence[float] = (1.0,) * 7,
        velocity_epsilon: float = 0.03,
    ):
        self.coulomb = _vector(coulomb, 7, "coulomb")
        self.viscous = _vector(viscous, 7, "viscous")
        self.max_torque = _vector(max_torque, 7, "max_torque")
        self.velocity_epsilon = float(velocity_epsilon)

    def __repr__(self) -> str:
        return (
            f"FrictionCompensationParams(coulomb={list(self.coulomb)}, "
            f"viscous={list(self.viscous)}, max_torque={list(self.max_torque)}, "
            f"velocity_epsilon={self.velocity_epsilon})"
        )


class JointReference:
    """A commanded joint reference for tracking motions.

    Unset components default to zero, as franky's do: a reference carrying only
    `q` still commands zero velocity and zero feedforward torque.
    """

    def __init__(
        self,
        q: Sequence[float] | None = None,
        dq: Sequence[float] | None = None,
        tau_ff: Sequence[float] | None = None,
    ):
        self.q = (0.0,) * 7 if q is None else _vector(q, 7, "q")
        self.dq = (0.0,) * 7 if dq is None else _vector(dq, 7, "dq")
        self.tau_ff = (0.0,) * 7 if tau_ff is None else _vector(tau_ff, 7, "tau_ff")

    def __repr__(self) -> str:
        return (
            f"JointReference(q={list(self.q)}, dq={list(self.dq)}, "
            f"tau_ff={list(self.tau_ff)})"
        )


class CartesianReference:
    """A commanded Cartesian reference for tracking motions."""

    def __init__(
        self,
        target: Any = None,
        target_twist: Any = None,
        target_acceleration: Any = None,
    ):
        self.target = Affine() if target is None else target
        self.target_twist = target_twist
        self.target_acceleration = target_acceleration

    def __repr__(self) -> str:
        return (
            f"CartesianReference(target={self.target!r}, target_twist={self.target_twist!r}, "
            f"target_acceleration={self.target_acceleration!r})"
        )
