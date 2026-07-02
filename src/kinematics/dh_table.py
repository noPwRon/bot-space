import yaml
from pathlib import Path


def load_robot(robot_name):
    # Resolves path relative to this file so the project works from any directory.
    config_path = Path(__file__).parent.parent.parent / "configs" / "robots" / robot_name
    # TODO: before opening, check whether config_path exists on disk.
    # If it doesn't, raise a FileNotFoundError with a message that shows the full path —
    # the default Python error won't tell the user where the code was looking.
    with open(config_path) as f:
        return yaml.safe_load(f), config_path
