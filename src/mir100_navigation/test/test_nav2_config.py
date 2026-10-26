from pathlib import Path

import yaml

from mir100_navigation.nav2_config import merge_nav2_params


CONFIG = Path(__file__).parents[1] / "config" / "nav2_params.yaml"


def test_nav2_selects_astar_and_collision_checked_pure_pursuit():
    overrides = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    base = {
        "planner_server": {"ros__parameters": {"GridBased": {"use_astar": False}}},
        "controller_server": {
            "ros__parameters": {"FollowPath": {"critics": [], "plugin": "old"}}
        },
    }

    merged = merge_nav2_params(base, overrides)
    controller = merged["controller_server"]["ros__parameters"]["FollowPath"]

    assert merged["planner_server"]["ros__parameters"]["GridBased"]["use_astar"]
    assert merged["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"]
    assert controller["plugin"] == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert controller["use_collision_detection"]
    assert controller["use_rotate_to_heading"]
    assert merged["collision_monitor"]["ros__parameters"]["observation_sources"] == ["scan"]
