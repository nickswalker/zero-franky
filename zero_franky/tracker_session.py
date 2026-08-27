"""
Tracker sessions are server-side objects managing impedance tracking motions.
"""
from __future__ import annotations

from dataclasses import dataclass
import importlib
import threading
import time
from types import SimpleNamespace
from typing import Any, Callable
import uuid

from zero_franky.protocol import decode_damping, encode_rpc_value

#: Floor for the timestep reported by policy steps.
_MIN_DT = 1e-9


class TrackerPolicyError(RuntimeError):
    pass


#: How long to wait for the server-side policy thread to notice the stop event.
#: Bounded independently of the motion join: the loop waits on the event, so it
#: exits promptly, and an unbounded wait here would let a wedged policy hang the
#: stop path.
POLICY_THREAD_JOIN_TIMEOUT = 1.0

_PREEMPTION_MESSAGE = "Move command preempted!"


def _is_preemption_exception(franky: Any, exc: BaseException) -> bool:
    """Whether this is libfranka's benign self-preemption error.

    Mirrors franky's own check (`franky.tracker._is_premption_exception`): a
    `ControlException` whose message names a preempted move command. A graceful
    :class:`franky.TorqueStopMotion` finishes without one, so this stays a defensive
    fallback
    for control ending abruptly before the stop could be enqueued.
    """
    control_exception = getattr(franky, "ControlException", None)
    if isinstance(control_exception, type) and not isinstance(exc, control_exception):
        return False
    return _PREEMPTION_MESSAGE in str(exc)


def _cartesian_gains_payload(
    gains: Any, translational_stiffness: float | None, rotational_stiffness: float | None, damping: Any
) -> dict[str, Any] | None:
    """Build the payload `_resolve_cartesian_gains` merges, or None for a no-op.

    A `gains` object carries `stiffness` and total-replaces; the decomposed form
    omits it so only the named blocks are overwritten.
    """
    stiffness_given = translational_stiffness is not None or rotational_stiffness is not None
    if gains is not None:
        if stiffness_given or damping is not None:
            raise ValueError(
                "Pass either gains (a full CartesianImpedanceGains, for anisotropic stiffness) "
                "or the isotropic translational_stiffness/rotational_stiffness/damping keywords, not both"
            )
        return {"stiffness": gains.stiffness, "damping": gains.damping}
    if not stiffness_given and damping is None:
        return None
    return {
        "stiffness": None,
        "translational_stiffness": translational_stiffness,
        "rotational_stiffness": rotational_stiffness,
        "damping": encode_rpc_value(damping),
    }


def load_policy(policy_payload: dict[str, Any]) -> Callable[[Any], Any]:
    transport = policy_payload["transport"]
    if transport == "import":
        module = importlib.import_module(policy_payload["module"])
        value = module
        for part in policy_payload["qualname"].split("."):
            value = getattr(value, part)
        return value
    if transport == "cloudpickle":
        import cloudpickle

        return cloudpickle.loads(policy_payload["payload"])
    raise TrackerPolicyError(f"Unsupported policy transport: {transport}")


@dataclass
class TrackerSessionStatus:
    id: str
    kind: str
    mode: str
    running: bool
    iterations: int
    error: str | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
            "mode": self.mode,
            "running": self.running,
            "iterations": self.iterations,
            "error": self.error,
        }


class TrackerSession:
    def __init__(
        self,
        *,
        franky: Any,
        robot: Any,
        kind: str,
        policy_factory: Callable[[Any], Any] | None,
        motion: Any,
        period: float | None = 0.001,
        stop_on_policy_error: bool = True,
    ):
        self.id = uuid.uuid4().hex
        self.kind = kind
        self._franky = franky
        self._robot = robot
        self._policy_factory = policy_factory
        self._motion = motion
        self._period = period
        self._stop_on_policy_error = stop_on_policy_error
        self._stop_event = threading.Event()
        self._stop_enqueued = False
        self._thread: threading.Thread | None = None
        self._iterations = 0
        self._error: str | None = None
        self._started_at = time.monotonic()
        self._dt = 0.0

    def start(self) -> str:
        if self._policy_factory is None:
            return self.id
        self._thread = threading.Thread(target=self._run_policy, name=f"zero-franky-{self.kind}-{self.id}", daemon=True)
        self._thread.start()
        return self.id

    def stop(self, join_timeout: float | None = None, stop_motion: Any = None):
        """Gracefully stop the tracker and wait for the arm to come to rest.

        Follows the corresponding Franky tracker’s
        :meth:`franky.JointImpedanceTracker.stop` /
        :meth:`franky.CartesianImpedanceTracker.stop` behavior: enqueue a
        motion. `join_timeout=None` blocks until the arm is at rest, as franky
        does; a float caps the wait, in which case the ramp may still be running
        when this returns.

        The join runs whether or not the robot is still in control, which is the
        point of it — a controller that already faulted has its exception stored,
        and joining is what surfaces it. Pass `stop_motion` to override the ramp.
        """
        self._stop_event.set()
        try:
            self._stop_robot(stop_motion, join_timeout)
        finally:
            if self._thread is not None and threading.current_thread() is not self._thread:
                self._thread.join(POLICY_THREAD_JOIN_TIMEOUT)

    def _request_stop(self):
        """Start the stop ramp without waiting for the arm to settle.

        For stops raised on the policy thread, which must not block in
        `join_motion`; the caller's `stop()` joins.
        """
        self._stop_event.set()
        self._enqueue_stop_motion(None)

    def _enqueue_stop_motion(self, stop_motion: Any):
        if stop_motion is None:
            if self._stop_enqueued:
                # Re-enqueueing would restart the ramp from the torque it has
                # already decayed to.
                return
            stop_motion_type = getattr(self._franky, "TorqueStopMotion", None)
            stop_motion = None if stop_motion_type is None else stop_motion_type()

        if not self._in_control:
            return
        self._stop_enqueued = True
        if stop_motion is None:
            # No graceful ramp available on this franky; stop abruptly.
            self._robot.stop()
        else:
            self._robot.move(stop_motion, asynchronous=True)

    def _stop_robot(self, stop_motion: Any, join_timeout: float | None):
        self._enqueue_stop_motion(stop_motion)

        join_motion = getattr(self._robot, "join_motion", None)
        if not callable(join_motion):
            return
        try:
            join_motion(join_timeout)
        except Exception as exc:
            if not _is_preemption_exception(self._franky, exc):
                raise

    @property
    def _in_control(self) -> bool:
        """Whether the robot is still executing this session's motion.

        `robot.move(..., asynchronous=True)` flips `is_in_control` before it
        returns, so this never reads False just because the RT loop has yet to
        spin up. Defaults True for a franky lacking the attribute.
        """
        return bool(getattr(self._robot, "is_in_control", True))

    def status(self) -> dict[str, Any]:
        """Session status, with `running` tied to the controller as franky's is.

        franky's `is_running` is `robot.is_in_control`, so `running` goes False
        when the controller faults, is preempted, or (torque) trips its signal
        watchdog — not only when the stop event is set or the thread exits.
        """
        in_control = self._in_control
        if self._policy_factory is None:
            running = in_control and not self._stop_event.is_set()
            mode = "passthrough"
        else:
            running = in_control and self._thread is not None and self._thread.is_alive()
            mode = "policy"
        return TrackerSessionStatus(self.id, self.kind, mode, running, self._iterations, self._error).as_dict()

    def set_joint_reference(
        self,
        position: list[float],
        velocity: list[float] | None = None,
        torque_feedforward: list[float] | None = None,
    ):
        from zero_franky.zmq_server_franky import _franky_joint_reference

        self._motion.set_reference(_franky_joint_reference(self._franky, position, velocity, torque_feedforward))

    def set_cartesian_reference(
        self, target: Any, target_twist: Any | None = None, target_acceleration: Any | None = None
    ):
        from zero_franky.zmq_server_franky import _franky_cartesian_reference

        self._motion.set_reference(_franky_cartesian_reference(self._franky, target, target_twist, target_acceleration))

    def set_joint_gains(self, stiffness: list[float] | None, damping: list[float] | str | None):
        """Retarget joint impedance gains.

        Naming neither gain changes nothing; an explicit unpin arrives as
        `CRITICAL_DAMPING`, not as a null. `stiffness=None` means "unchanged" and
        is read back, since `JointImpedanceGains(None, ...)` would reset it to
        franky's 50 Nm/rad default. A null damping unpins, so a stiffness change
        re-criticals unless damping is named too.
        """
        if stiffness is None and damping is None:
            return
        if stiffness is None:
            stiffness = self._motion.get_gains().stiffness
        self._motion.set_gains(self._franky.JointImpedanceGains(stiffness, decode_damping(damping)))

    def set_cartesian_gains(self, payload: dict[str, Any]):
        """Retarget Cartesian impedance gains, overwriting only the named blocks."""
        gains = self._resolve_cartesian_gains(self._motion.get_gains, payload)
        if gains is not None:
            self._motion.set_gains(gains)

    def set_hybrid_cartesian_gains(self, payload: dict[str, Any]):
        """Retarget the hybrid Cartesian gain shaping on a joint impedance motion."""
        gains = self._resolve_cartesian_gains(self._motion.get_cartesian_gains, payload)
        if gains is not None:
            self._motion.set_cartesian_gains(gains)

    def _resolve_cartesian_gains(self, read_current: Callable[[], Any], payload: dict[str, Any]):
        """Apply a Cartesian gains payload on top of the motion's current gains.

        A payload carrying `stiffness` is a full encoded `CartesianImpedanceGains`
        and total-replaces. Otherwise only the
        named 3x3 stiffness blocks are overwritten, leaving anisotropy and the
        unnamed block intact.

        A payload naming nothing returns None, the no-op; an explicit unpin names
        damping with the sentinel.
        """
        from zero_franky.zmq_server_franky import _franky_cartesian_gains

        if all(
            payload.get(key) is None
            for key in ("stiffness", "translational_stiffness", "rotational_stiffness", "damping")
        ):
            return None

        if payload.get("stiffness") is not None:
            return _franky_cartesian_gains(self._franky, payload)

        import numpy as np

        current = read_current()
        stiffness = np.array(current.stiffness, copy=True)
        translational = payload.get("translational_stiffness")
        rotational = payload.get("rotational_stiffness")
        if translational is not None:
            stiffness[0:3, 0:3] = float(translational) * np.eye(3)
        if rotational is not None:
            stiffness[3:6, 3:6] = float(rotational) * np.eye(3)
        current.stiffness = stiffness
        # Unpinned damping re-tracks critical against the new stiffness.
        damping = decode_damping(payload.get("damping"))
        current.damping = None if damping is None else np.diag(np.asarray(damping, dtype=float))
        return current

    def set_nullspace_gains(self, gains: Any):
        self._motion.set_nullspace_gains(gains)

    def set_posture_stiffness(self, posture_stiffness: Any):
        """Nudge only the posture task's stiffness, leaving the other nullspace gains intact."""
        gains = self._motion.get_nullspace_gains()
        gains.posture_stiffness = posture_stiffness
        self._motion.set_nullspace_gains(gains)

    def set_torque(self, torque: list[float]):
        if self.kind != "torque":
            raise RuntimeError("set_torque is only valid for torque sessions")
        self._motion.set_torque(torque)

    def get_torque(self):
        if self.kind != "torque":
            raise RuntimeError("get_torque is only valid for torque sessions")
        return self._motion.get_torque()

    def _handle(self) -> _TrackerHandle:
        """The handle a policy step drives its own tracker through."""
        return TRACKER_HANDLE_TYPES.get(self.kind, _TrackerHandle)(self)

    def _run_policy(self):
        if self._policy_factory is None:
            return
        ctx = _TrackerContext(
            franky=self._franky,
            robot=self._robot,
            kind=self.kind,
            session_id=self.id,
            stop_event=self._stop_event,
            tracker=self._handle(),
        )
        try:
            candidate = self._policy_factory(ctx)
            if callable(candidate):
                policy = candidate
            else:
                self._apply_reference(candidate)
                policy = self._policy_factory

            next_time = time.monotonic()
            t_last = next_time
            # `_in_control` is this loop's `tick()`: end when the controller does,
            # rather than streaming references into a faulted motion.
            while not self._stop_event.is_set() and self._in_control:
                now = time.monotonic()
                if self._iterations == 0:
                    # No previous step to measure against, so report the cycle
                    # about to run, as franky's first `tick()` does.
                    dt = self._period if self._period is not None else _MIN_DT
                else:
                    dt = max(now - t_last, _MIN_DT)
                t_last = now
                self._dt = dt
                ctx.dt = dt
                ctx.elapsed = now - ctx.started_at
                ctx.iterations = self._iterations
                reference = policy(ctx)
                self._apply_reference(reference)
                self._iterations += 1

                if self._period is None:
                    # Unpaced: run the policy as fast as the loop allows.
                    continue

                next_time += self._period
                sleep_time = next_time - time.monotonic()
                if sleep_time > 0:
                    self._stop_event.wait(sleep_time)
                else:
                    next_time = time.monotonic()
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            if self._stop_on_policy_error:
                # Ramp rather than preempt: the impedance motions have no
                # watchdog, so a dead policy leaves the arm held at its last
                # reference, and a hard stop is the discontinuity
                # TorqueStopMotion exists to avoid.
                self._request_stop()

    def _apply_reference(self, reference: Any):
        if reference is None:
            return
        if reference is False:
            self._request_stop()
            return
        if not isinstance(reference, dict):
            raise TrackerPolicyError("Policy step must return a dict, None, or False")
        if reference.get("stop"):
            self._request_stop()
            return
        if self.kind == "joint":
            self.set_joint_reference(
                reference["position"],
                reference.get("velocity"),
                reference.get("torque_feedforward"),
            )
            return
        if self.kind == "cartesian":
            self.set_cartesian_reference(
                reference["target"],
                reference.get("target_twist"),
                reference.get("target_acceleration"),
            )
            return
        raise TrackerPolicyError(f"Unsupported tracker kind: {self.kind}")


class _TrackerHandle:
    """A policy's view of its own tracker, named as `franky`'s tracker classes are.

    No `stop()`: joining the ramp on the policy thread would leave the caller's
    `stop()` waiting on that thread. Use `request_stop()`, or return `False` or
    `{"stop": True}` from the step.
    """

    def __init__(self, session: "TrackerSession"):
        self._session = session

    @property
    def id(self) -> str:
        return self._session.id

    @property
    def kind(self) -> str:
        return self._session.kind

    @property
    def is_running(self) -> bool:
        """Whether the controller is still active, as `franky`'s `is_running` is."""
        return bool(self._session.status()["running"])

    def status(self) -> dict[str, Any]:
        return self._session.status()

    @property
    def period(self) -> float | None:
        """The configured loop period in seconds, or None if the tracker is unpaced."""
        return self._session._period

    @property
    def dt(self) -> float:
        """The timestep most recently returned by tick (0.0 before first step)."""
        return self._session._dt

    @property
    def tick_count(self) -> int:
        """Number of ticks that have run."""
        return self._session._iterations

    @property
    def elapsed_time(self) -> float:
        """Seconds since this tracker was created."""
        return time.monotonic() - self._session._started_at

    def request_stop(self) -> None:
        """Start the stop ramp and end the policy loop, without waiting."""
        self._session._request_stop()


class _JointTrackerHandle(_TrackerHandle):
    """The streaming surface of `franky.JointImpedanceTracker`."""

    def set_target(self, q: Any, dq: Any = None, tau_ff: Any = None) -> None:
        """Update the joint target position, optional velocity, and optional feedforward torque."""
        self._session.set_joint_reference(q, dq, tau_ff)

    def set_gains(self, *, stiffness: Any = None, damping: Any = None) -> None:
        """Update joint impedance gains, smoothed in the RT loop by the controller.

        Damping is all-or-nothing: a 7-vector pins it, `franky.CRITICAL` unpins
        it, omitting it re-criticals. Omitting both is a no-op — distinct from
        `damping=CRITICAL`, which unpins and leaves stiffness alone.
        """
        self._session.set_joint_gains(stiffness, encode_rpc_value(damping))

    def set_cartesian_gains(
        self,
        gains: Any = None,
        *,
        translational_stiffness: float | None = None,
        rotational_stiffness: float | None = None,
        damping: Any = None,
    ) -> None:
        """Retune the hybrid Cartesian gain shaping, if the motion was built with it."""
        payload = _cartesian_gains_payload(gains, translational_stiffness, rotational_stiffness, damping)
        if payload is not None:
            self._session.set_hybrid_cartesian_gains(payload)


class _CartesianTrackerHandle(_TrackerHandle):
    """The streaming surface of `franky.CartesianImpedanceTracker`."""

    def set_target(self, pose: Any, twist: Any = None, acceleration: Any = None) -> None:
        """Update the Cartesian target pose and optional twist/acceleration feedforward."""
        self._session.set_cartesian_reference(pose, twist, acceleration)

    def set_gains(
        self,
        gains: Any = None,
        *,
        translational_stiffness: float | None = None,
        rotational_stiffness: float | None = None,
        damping: Any = None,
        posture_stiffness: Any = None,
        nullspace_gains: Any = None,
    ) -> None:
        """Update Cartesian impedance gains, smoothed in the RT loop by the controller.

        `translational_stiffness`/`rotational_stiffness` overwrite one stiffness
        block each, leaving anisotropy intact; `gains` total-replaces and is
        exclusive with them. `posture_stiffness` and `nullspace_gains` retune the
        nullspace and are mutually exclusive.
        """
        if nullspace_gains is not None and posture_stiffness is not None:
            raise ValueError("Pass either nullspace_gains or posture_stiffness, not both")

        payload = _cartesian_gains_payload(gains, translational_stiffness, rotational_stiffness, damping)
        if payload is not None:
            self._session.set_cartesian_gains(payload)
        if nullspace_gains is not None:
            self._session.set_nullspace_gains(nullspace_gains)
        elif posture_stiffness is not None:
            self._session.set_posture_stiffness(posture_stiffness)

    def set_nullspace_gains(self, gains: Any) -> None:
        """Replace the whole `NullspaceGains` acting on the redundant joint."""
        self._session.set_nullspace_gains(gains)


TRACKER_HANDLE_TYPES = {"joint": _JointTrackerHandle, "cartesian": _CartesianTrackerHandle}


class _TrackerContext(SimpleNamespace):
    def __init__(
        self, *, franky, robot, kind: str, session_id: str, stop_event: threading.Event, tracker: Any = None
    ):
        super().__init__(
            franky=franky,
            robot=robot,
            kind=kind,
            session_id=session_id,
            tracker=tracker,
            started_at=time.monotonic(),
            elapsed=0.0,
            iterations=0,
            dt=0.0,
        )
        self._stop_event = stop_event

    @property
    def period(self) -> float | None:
        """The configured loop period in seconds, or None if unpaced."""
        return self.tracker.period if self.tracker is not None else None

    @property
    def elapsed_time(self) -> float:
        """`elapsed`, under franky's name for it."""
        return self.elapsed

    @property
    def tick_count(self) -> int:
        """`iterations`, under franky's name for it."""
        return self.iterations

    @property
    def is_running(self) -> bool:
        return self.tracker is not None and self.tracker.is_running

    @property
    def stop_requested(self) -> bool:
        return self._stop_event.is_set()

    def stop(self) -> dict[str, bool]:
        return {"stop": True}
