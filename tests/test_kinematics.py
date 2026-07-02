# Reference: pytest basics — https://docs.pytest.org/en/stable/getting-started.html
# Reference: Craig, "Introduction to Robotics", Ch. 3 (DH transforms)

import numpy as np
from sympy import pi
from src.kinematics.transforms import build_dh_matrix, build_T_matrices
from src.kinematics.inverse_kinematics import ik_newton_raphson
from src.kinematics.jacobian import build_end_effector_jacobian
from src.controller.controller import make_forward_kin_fn, make_jacobian_fn, numbify
from tests.conftest import assert_close



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
    

    target_T = np.eye(4)
    target_T[0,3] = 2

    theta_init = np.array([1,1])


    forward_kin_fn = make_forward_kin_fn(joint)
    jacobian_fn = make_jacobian_fn(joint)
    new_theta = ik_newton_raphson(target_T, theta_init, forward_kin_fn,jacobian_fn)
    
    # print(new_theta)
    assert_close(forward_kin_fn(new_theta)[:3,3], np.array([2,0,0]))


