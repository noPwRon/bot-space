import yaml
from pathlib import Path

# loading .yaml data

def load_robot(robot_name):
    config_path = Path(__file__).parent.parent.parent / "configs" / "robots" / robot_name
    with open(config_path) as f:
        return yaml.safe_load(f)

# data = load_robot(robot_name)

# print(data["joints"][0]["a"])

