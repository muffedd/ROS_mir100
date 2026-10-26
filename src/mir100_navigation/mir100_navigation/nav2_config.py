"""Small typed YAML merge for the Nav2 parameter overlay."""

from copy import deepcopy


def merge_nav2_params(base: dict, overrides: dict) -> dict:
    """Return base parameters with nested override values applied."""
    merged = deepcopy(base)
    _merge_in_place(merged, overrides)
    return merged


def _merge_in_place(target: dict, overrides: dict) -> None:
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge_in_place(target[key], value)
        else:
            target[key] = deepcopy(value)
