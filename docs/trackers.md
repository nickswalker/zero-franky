# Trackers

A client-side tracker mirrors `franky` and sends references over the conflated tracker socket:

```python
import time

with robot.start_joint_impedance_tracker(stiffness=[10.0] * 7) as tracker:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and tracker.tick() is not None:
        tracker.set_target(q, dq=dq)
```

Leaving the `with` block stops the tracker. `tick()` returns `None` when control ends.

For lower latency, run the policy beside `franky` on the server:

- `policy_transport="import"`: the policy must be installed as a module on the server.
- `policy_transport="cloudpickle"`: the policy is serialized and sent over RPC.

A policy step returns a joint or Cartesian reference, `None` to hold the current reference, or `False` / `{"stop": True}` to stop.
