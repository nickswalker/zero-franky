from __future__ import annotations

from dataclasses import dataclass
import importlib
import threading
import time
from types import SimpleNamespace
from typing import Any, Callable
import uuid


class TrackerPolicyError(RuntimeError):
    pass


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
    running: bool
    iterations: int
    error: str | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
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
        policy_factory: Callable[[Any], Any],
        reference_handle: Any,
        period: float,
        stop_on_policy_error: bool,
    ):
        self.id = uuid.uuid4().hex
        self.kind = kind
        self._franky = franky
        self._robot = robot
        self._policy_factory = policy_factory
        self._reference_handle = reference_handle
        self._period = period
        self._stop_on_policy_error = stop_on_policy_error
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._iterations = 0
        self._error: str | None = None

    def start(self) -> str:
        self._thread = threading.Thread(target=self._run_policy, name=f"zero-franky-{self.kind}-{self.id}", daemon=True)
        self._thread.start()
        return self.id

    def stop(self, join_timeout: float | None = 1.0):
        self._stop_event.set()
        try:
            self._robot.stop()
        finally:
            if self._thread is not None and threading.current_thread() is not self._thread:
                self._thread.join(join_timeout)

    def status(self) -> dict[str, Any]:
        running = self._thread is not None and self._thread.is_alive()
        return TrackerSessionStatus(self.id, self.kind, running, self._iterations, self._error).as_dict()

    def set_joint_reference(
        self,
        position: list[float],
        velocity: list[float] | None = None,
        torque_feedforward: list[float] | None = None,
    ):
        self._reference_handle.set(position, velocity, torque_feedforward)

    def set_cartesian_reference(self, target: Any, target_twist: Any | None = None):
        self._reference_handle.set(target, target_twist)

    def _run_policy(self):
        ctx = _TrackerContext(
            franky=self._franky,
            robot=self._robot,
            kind=self.kind,
            session_id=self.id,
            stop_event=self._stop_event,
        )
        try:
            candidate = self._policy_factory(ctx)
            if callable(candidate):
                policy = candidate
            else:
                self._apply_reference(candidate)
                policy = self._policy_factory

            next_time = time.monotonic()
            while not self._stop_event.is_set():
                now = time.monotonic()
                ctx.elapsed = now - ctx.started_at
                ctx.iterations = self._iterations
                reference = policy(ctx)
                self._apply_reference(reference)
                self._iterations += 1

                next_time += self._period
                sleep_time = next_time - time.monotonic()
                if sleep_time > 0:
                    self._stop_event.wait(sleep_time)
                else:
                    next_time = time.monotonic()
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            if self._stop_on_policy_error:
                self._robot.stop()

    def _apply_reference(self, reference: Any):
        if reference is None:
            return
        if reference is False:
            self._stop_event.set()
            self._robot.stop()
            return
        if not isinstance(reference, dict):
            raise TrackerPolicyError("Policy step must return a dict, None, or False")
        if reference.get("stop"):
            self._stop_event.set()
            self._robot.stop()
            return
        if self.kind == "joint":
            self._reference_handle.set(
                reference["position"],
                reference.get("velocity"),
                reference.get("torque_feedforward"),
            )
            return
        if self.kind == "cartesian":
            self._reference_handle.set(reference["target"], reference.get("target_twist"))
            return
        raise TrackerPolicyError(f"Unsupported tracker kind: {self.kind}")


class _TrackerContext(SimpleNamespace):
    def __init__(self, *, franky, robot, kind: str, session_id: str, stop_event: threading.Event):
        super().__init__(
            franky=franky,
            robot=robot,
            kind=kind,
            session_id=session_id,
            started_at=time.monotonic(),
            elapsed=0.0,
            iterations=0,
        )
        self._stop_event = stop_event

    @property
    def stop_requested(self) -> bool:
        return self._stop_event.is_set()

    def stop(self) -> dict[str, bool]:
        return {"stop": True}
