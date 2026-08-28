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

Every control cycle, the robot produces a ~2.5 KB RobotState object. Depending on your network configuration, this might
be too much information to send every control cycle. So zero_franky's `tracker.state` and `robot.latest_state` are specialized msgpack dictionaries which only contain subsets of the full
[`franky.RobotState`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html) values.

The default is the `fast` profile, which contains:

| Key | Meaning |
|---|---|
| [`q`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html#franky.RobotState.q), [`dq`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html#franky.RobotState.dq) | Measured joint position and velocity. |
| [`O_T_EE`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html#franky.RobotState.O_T_EE) | Measured end-effector pose. |
| [`O_F_ext_hat_K`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html#franky.RobotState.O_F_ext_hat_K), [`K_F_ext_hat_K`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html#franky.RobotState.K_F_ext_hat_K), [`tau_ext_hat_filtered`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html#franky.RobotState.tau_ext_hat_filtered) | Force/torque estimates. |
| [`O_dP_EE_est`](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html#franky.RobotState.O_dP_EE_est) | Estimated end-effector twist. |
| `time_step` | Duration of the last control cycle, in seconds. |
| `rel_time` | Time elapsed since the motion started, in seconds. |
| `abs_time` | Robot time, in seconds. |
| `control_signal` | Motion-specific control signal produced by `franky`. |
| `robot_id` | Server-assigned identifier for the robot proxy. |
| `in_control` | Whether the robot is still executing the motion. |
| `reference` | Latest reference accepted by the controller; motion-specific. |

`reference` is `JointReference`, `CartesianReference`, or `Torque`, with a
motion-specific payload.

## Accessing Extended State Information

Use `full` to stream all 54 `RobotState` fields, or specify a comma-separated list of fields:

```bash
zero-franky server --state-fields full
zero-franky server --state-fields q,dq,O_T_EE
```

For lower latency, run the policy beside `franky` on the server:

- `policy_transport="import"`: the policy must be installed as a module on the server.
- `policy_transport="cloudpickle"`: the policy is serialized and sent over RPC.

A policy step returns a joint or Cartesian reference, `None` to hold the current reference, or `False` / `{"stop": True}` to stop.
