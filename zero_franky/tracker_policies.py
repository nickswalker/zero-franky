from __future__ import annotations


def hold_current_joint(context):
    position = list(context.robot.current_joint_positions)
    velocity = [0.0] * len(position)

    def step(_context):
        return {"position": position, "velocity": velocity}

    return step


def hold_current_cartesian(context):
    target = context.robot.current_pose.end_effector_pose

    def step(_context):
        return {"target": target}

    return step


def passthrough_cartesian(_context):
    """Policy for external reference streaming (e.g. teleop).

    The server initialises the reference handle to the current pose before the
    policy thread starts, so returning None every step lets set_cartesian_reference
    RPC calls drive the motion without being overwritten by the policy.
    """
    def step(_context):
        return None

    return step


def passthrough_joint(_context):
    """Joint-space equivalent of passthrough_cartesian."""
    def step(_context):
        return None

    return step
