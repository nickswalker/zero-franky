# zero-franky

Use [`franky`](https://github.com/TimSchneider42/franky) from another process over ZeroMQ.

* Thin client: doesn't require libfranka, nor RT kernel, nor Python-version match with the server
* Optional proxy for Robotiq grippers

```
 [FRANKA ARM] <----> [CONTROL BOX] <--LAN--> [RT CONTROL PC] <--ZeroMQ--> [APP COMPUTER]
                                             franky + libfranka          your code: policy,
                                             zero-franky server          ML, ROS, any Python
```

zero-franky was inspired by [net-franky](https://github.com/yblei/net_franky), but adopts a lower level of wrapping and command serialization to make it possible to run control loops much faster (500hz instead of ~5hz).

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

`Robot` is a proxy. Motions are encoded into compact [msgpack](https://msgpack.org/index.html) payloads, and the server reconstructs real `franky` objects next to the robot. You can also pass most objects from `franky` straight into zero franky and they'll serialize the same way.


## Server

On the robot host:

```bash
zero-franky server
```

By default, this binds three TCP sockets on `0.0.0.0`: 18812 is used for blocking request-response style remote procedure calls (RPC), 18813 is dedicated to the stream of state information that the robot publishes; and 18814 is used for clients to send cartesian/joint references during [tracking motions](#impedance-trackers).

You can change the host or the base port if you like:

```bash
zero-franky server --host 192.168.1.20 --port 18812
```

### Robotiq gripper

If you'd like to use Robotiq grippers, install the additional dependencies:

```bash
pip install 'zero-franky[server,robotiq]'
```

Run the robot server together with the gripper server with `--robotiq`:

```bash
zero-franky server --robotiq --com-port auto
```

Or run the gripper service on its own with `zero-franky gripper serve --com-port auto`.

The `gripper` subcommand also provides diagnostic client commands:

```bash
zero-franky gripper status --host control-machine
zero-franky gripper open --host control-machine
```


## Impedance Trackers

Zero Franky wraps the franky's `JointImpedanceTrackingMotion` and `CartesianImpedanceTrackingMotion` (client-side torque controllers for trajectory tracking or complex control) specially to reduce network round trips. `robot.start_joint_impedance_tracker` returns a proxy that mirrors franky's in-process `JointImpedanceTracker` and `CartesianImpedanceTracker`, so loop bodies should port over unchanged:

```python
with robot.start_joint_impedance_tracker(stiffness=[10.0] * 7, damping=[6.0] * 7, period=0.01) as tracker:
    while dt := tracker.tick():
        tracker.set_target(q, dq=dq)
```

The proxy stops the tracker when the context block exits.

`tick()` paces the loop and returns `None` when the controller dies. Without a `period`, you can drive the loop at whatever rate the network allows. `set_target` goes over a conflating socket that may drop an update in favour of a newer one.

If you need to run a control policy with the lowest possible latency, you'll need to transport the policy to run on the server side. There are two policy transports:

- `import`: send `module` + `qualname`; the server imports the policy. Use this for stable policies installed on the robot host.
- `cloudpickle`: serialize the function and send it over RPC. 

### Imported Policies

Here's an example default "policy" that simply holds the current joint configuration with a certain stiffness.

```python
import time

from zero_franky.tracker_policies import hold_current_joint

with robot.start_joint_impedance_tracker(
    hold_current_joint,
    stiffness=[10.0] * 7
) as tracker:
    deadline = time.monotonic() + 10.0
    status = tracker.status()
    while time.monotonic() < deadline and tracker.tick() is not None:
        pass
```

### Pickled Policies

Or it can be shipped with `cloudpickle` for exploratory work:

```python
import math
import time


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
    deadline = time.monotonic() + 10.0
    status = tracker.status()
    while time.monotonic() < deadline and tracker.tick() is not None:
        pass
```


The policy function receives a context with `franky`, `robot`, `elapsed`, `iterations`, `stop()`, and `tracker`, a handle onto its own tracker carrying franky's `set_target`/`set_gains`. A factory may return a step function, or the policy may act directly as the step function. Joint steps return `{"position": q, "velocity": dq, "torque_feedforward": tau}`. Cartesian steps return `{"target": affine, "target_twist": twist}`.
