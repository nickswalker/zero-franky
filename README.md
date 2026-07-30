# zero-franky

Use [`franky`](https://github.com/TimSchneider42/franky) from another process over ZeroMQ.

* Thin client: doesn't require libfranka, nor RT kernel, nor Python-version match with the server
* Optional proxy for Robotiq grippers

```
 [FRANKA ARM] <----> [CONTROL BOX] <--LAN--> [RT CONTROL PC] <--ZeroMQ--> [APP COMPUTER]
                                             franky + libfranka          your code: policy,
                                             zero-franky server          ML, ROS, any Python
```

zero-franky was inspired by [net-franky](https://github.com/yblei/net_franky), but adopts a lower level of wrapping and command serialization to make it possible to run control loops at 500hz instead of ~5hz.

## Usage

```python
from zero_franky import setup_zero_franky
from zero_franky import Robot
from zero_franky.types import Affine, CartesianMotion, ReferenceType

setup_zero_franky("server-ip", 18812)

robot = Robot("192.168.100.1")
motion = CartesianMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)
robot.move(motion, asynchronous=True)
robot.join_motion()
```

`Robot` is a proxy. Motions are encoded into plain msgpack payloads, and the server reconstructs real `franky` objects next to the robot. You can also pass most objects from `franky` straight into zero franky and they'll serialize the same way.


## Server

On the robot host:

```bash
zero-franky server
```

By default this binds RPC on `tcp://0.0.0.0:18812`, state PUB on `tcp://0.0.0.0:18813`, and tracker updates on `tcp://0.0.0.0:18814`.

Common overrides:

```bash
zero-franky server --host 192.168.1.20 --port 18812
zero-franky server --port 19000 --no-pub
```

### Robotiq gripper

Robotiq 2F-85 support is optional. Install both control-machine extras only on hosts
that physically run the robot and gripper services:

```bash
pip install 'zero-franky[server,robotiq]'
```

The base installation contains the network client and does not import or
require `pyrobotiqgripper`.

Run the robot server together with the gripper server with `--robotiq`:

```bash
zero-franky server --robotiq --com-port auto
```

Or run the gripper service on its own (e.g. on a different host, or the robot
server is already running separately):

```bash
zero-franky gripper serve --com-port auto
```

The `gripper` subcommand also provides diagnostic client commands:

```bash
zero-franky gripper status --host control-machine
zero-franky gripper open --host control-machine
```

The equivalent Python entry point is:

```python
from zero_franky.zmq_server import ZmqRobotServer

ZmqRobotServer(
    bind="tcp://0.0.0.0:18812",
    pub_bind="tcp://0.0.0.0:18813",
    tracker_bind="tcp://0.0.0.0:18814",
).serve_forever()
```

## Impedance Trackers

Franky supports client-side torque controllers (low level controllers that run in user code, not inside the Franka control box) via `JointImpedanceTrackingMotion` and `CartesianImpedanceTrackingMotion`. These are useful if you need to track trajectories or do more complex control.

Zero Franky wraps the trackers specially to reduce network round trips (avoiding `tick()` and batch requesting all robot state information). The impedance motion and reference handle stay on the robot host. Otherwise the returned proxy mirrors franky's in-process `JointImpedanceTracker` and `CartesianImpedanceTracker`, so loop bodies port over unchanged:

```python
with robot.start_joint_impedance_tracker(stiffness=[10.0] * 7, damping=[6.0] * 7) as tracker:
    tracker.set_target(q, dq=dq)
    tracker.set_gains(stiffness=[20.0] * 7, damping=[8.0] * 7)
```

The proxy stops the tracker when the context block exits.

Two departures from franky's trackers, both because the loop is remote. There is no `tick()`, so drive the loop from client code at whatever rate the network allows (or hand the server a policy, below). And `is_running` / `iterations` each cost an RPC round trip.

One addition, for the same reason: `set_target` goes over a conflating socket that may drop an update in favour of a newer one, so `tracker.last_reference` reports what the controller actually picked up. It comes from the state the server publishes each control cycle, so reading it costs no round trip — as do `current_joint_positions` and `current_pose`.

If you need to run a control policy with the lowest possible latency, you'll need to transport the policy to run on the server side. There are two policy transports:

- `import`: send `module` + `qualname`; the server imports the policy. Use this for stable policies installed on the robot host.
- `cloudpickle`: serialize the function and send it over RPC. 

### Imported Policies

Here's an example default "policy" that simply holds the current joint configuration with a certain stiffness.

```python
from zero_franky.tracker_policies import hold_current_joint

with robot.start_joint_impedance_tracker(
    hold_current_joint,
    stiffness=[10.0] * 7
) as tracker:
    status = tracker.status()
```

### Pickled Policies

Or it can be shipped with `cloudpickle` for exploratory work:

```python
import math


def wiggle_joints(context):
    q = list(context.robot.current_joint_positions)
    amplitude = 0.03
    frequency = 0.25
    phase_offsets = [index * math.pi / 7.0 for index in range(7)]

    def step(context):
        omega = 2.0 * math.pi * frequency
        position = [
            q_i + amplitude * math.sin(omega * context.elapsed + phase)
            for q_i, phase in zip(q, phase_offsets)
        ]
        velocity = [
            amplitude * omega * math.cos(omega * context.elapsed + phase)
            for phase in phase_offsets
        ]
        return {"position": position, "velocity": velocity}

    return step

with robot.start_joint_impedance_tracker(
    wiggle_joints,
    policy_transport="cloudpickle",
    stiffness=[10.0] * 7,
) as tracker:
    status = tracker.status()
```


The policy function receives a context with `franky`, `robot`, `elapsed`, `iterations`, `stop()`, and `tracker`, a handle onto its own tracker carrying franky's `set_target`/`set_gains`. A factory may return a step function, or the policy may act directly as the step function. Joint steps return `{"position": q, "velocity": dq, "torque_feedforward": tau}`. Cartesian steps return `{"target": affine, "target_twist": twist}`.
