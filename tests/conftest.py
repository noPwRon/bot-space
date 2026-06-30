import pytest
from src.kinematics.transforms import build_T_matrices, build_cumulative_transforms
from src.controller.controller import build_theta_syms, build_theta_dot_syms
import numpy as np

@pytest.fixture
def joint_build():
    joint = [{"type": "revolute", "a": 1, "alpha": 0, "d": 0, "theta": 0, "mass": 1, "length": 1, "geometry": "box", "width": 1, "height": 1}]
    theta_syms = build_theta_syms(joint)
    theta_dot_syms = build_theta_dot_syms(joint)
    T = build_T_matrices(joint)
    T_cum = build_cumulative_transforms(T)
    return joint, theta_syms, theta_dot_syms, T_cum


def assert_close(num_result, expected_result):
    assert np.allclose(num_result, expected_result)
