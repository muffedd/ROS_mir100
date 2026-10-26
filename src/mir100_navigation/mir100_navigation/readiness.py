"""Pure readiness checks for the one-shot Nav2 goal sender."""

# lifecycle_msgs/msg/State primary state id for an active lifecycle node.
PRIMARY_STATE_ACTIVE = 3


def is_active_state(state_id: int) -> bool:
    """Return True when a lifecycle get_state response means active."""
    return state_id == PRIMARY_STATE_ACTIVE


def nav2_goal_ready(map_ready: bool, server_ready: bool, nav2_active: bool) -> bool:
    """A goal may be sent only with a live map, the action server, and active Nav2."""
    return map_ready and server_ready and nav2_active
