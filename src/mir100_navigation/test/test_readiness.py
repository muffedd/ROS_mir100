from mir100_navigation.readiness import (
    PRIMARY_STATE_ACTIVE,
    is_active_state,
    nav2_goal_ready,
)


def test_active_state_id_matches_lifecycle_primary_state_active():
    assert PRIMARY_STATE_ACTIVE == 3
    assert is_active_state(3)
    assert not is_active_state(2)  # inactive
    assert not is_active_state(1)  # unconfigured
    assert not is_active_state(0)  # unknown


def test_goal_ready_requires_map_server_and_active_nav2():
    assert nav2_goal_ready(True, True, True)
    assert not nav2_goal_ready(False, True, True)
    assert not nav2_goal_ready(True, False, True)
    assert not nav2_goal_ready(True, True, False)
