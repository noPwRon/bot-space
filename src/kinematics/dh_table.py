import yaml
from pathlib import Path


def load_robot(robot_name):
    # Resolves path relative to this file so the project works from any directory.
    config_path = Path(__file__).parent.parent.parent / "configs" / "robots" / robot_name
    with open(config_path) as f:
        return yaml.safe_load(f)
