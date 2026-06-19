# Reference: pytest basics — https://docs.pytest.org/en/stable/getting-started.html
# Reference: Craig, "Introduction to Robotics", Ch. 3 (DH transforms)

import numpy as np
from sympy import pi
from src.kinematics.transforms import build_dh_matrix, build_T_matrices
from src.kinematics.inverse_kinematics import ik_newton_raphson


def numbify(sym_result):
    return np.array(sym_result.evalf(), dtype=float)


def assert_close(num_result, expected_result):
    assert np.allclose(num_result, expected_result)


# --- Test 1: identity transform ---

def test_eyeTransform():
    sym_result = build_dh_matrix(0, 0, 0, 0)

    num_result = numbify(sym_result=sym_result)

    expected_result = np.eye(4)

    assert_close(num_result, expected_result)


# --- Test 2: pure translation ---

def test_pureTranslation():
    sym_result = build_dh_matrix(1, 0, 0, 0)

    num_result = numbify(sym_result=sym_result)

    expected_result = np.eye(4)
    expected_result[0, 3] = 1

    assert_close(num_result, expected_result)


# --- Test 3: pure rotation ---

def test_pureRotation():
    sym_result = build_dh_matrix(0, 0, 0, pi/2)

    num_result = numbify(sym_result=sym_result)

    expected_result = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    assert_close(num_result, expected_result)


# --- Test 4: chaining two joints ---

def test_twoJoint():
    t_1 = build_dh_matrix(1, 0, 0, 0)
    t_2 = build_dh_matrix(1, 0, 0, 0)
    sym_result = t_1 @ t_2

    num_result = numbify(sym_result=sym_result)

    expected_result = np.eye(4)
    expected_result[0, 3] = 2

    assert_close(num_result, expected_result)

def test_ikSolution(joint_build):
    joint, theta_syms, theta_dot_syms, T_cum = joint_build

    # TODO: build forward_kin_fn using make_forward_kin_fn(joint)
    # TODO: build jacobian_fn — numerical Jacobian callable
    # TODO: call ik_newton_raphson with target_T, initial offset theta, forward_kin_fn, jacobian_fn
    # TODO: assert returned theta is close to 0

