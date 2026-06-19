import sympy as smp

# Reference: Craig, Introduction to Robotics, Ch. 3
# Reference: CMU 16-311 Lecture 16b — https://www.cs.cmu.edu/~16311/f05/lecture/lec16b.html
# Reference: Lynch & Park, Modern Robotics, Ch. 4


def build_dh_matrix(a, alpha, d, theta):
    # Returns the 4x4 modified DH transformation matrix ^(i-1)T_i for one joint.
    return smp.Matrix([
        [smp.cos(theta),                -smp.sin(theta),                 0,              a],
        [smp.sin(theta)*smp.cos(alpha),  smp.cos(theta)*smp.cos(alpha), -smp.sin(alpha), -d*smp.sin(alpha)],
        [smp.sin(theta)*smp.sin(alpha),  smp.cos(theta)*smp.sin(alpha),  smp.cos(alpha),  d*smp.cos(alpha)],
        [0,                              0,                               0,              1]
    ])


def build_T_matrices(joints):
    # Returns a list of symbolic T matrices, one per joint.
    # Accepts a joints list directly — pass data["joints"] or the output of build_symbolic_joints.
    T = []
    for joint in joints:
        T.append(build_dh_matrix(joint["a"], joint["alpha"], joint["d"], joint["theta"]))
    return T


def build_end_effector_T(T):
    # Returns ^0T_n by sequentially multiplying all joint T matrices left to right.
    # result[:3, 3] is end-effector position; result[:3, :3] is end-effector orientation.
    big_T = smp.eye(4)
    for joint in T:
        big_T = big_T * joint
    return big_T


def build_cumulative_transforms(T):
    # Returns a list of n cumulative transforms [^0T_1, ^0T_2, ..., ^0T_n].
    # Index k of the result is the transform from the base frame to link k+1.
    # This is needed any time you want the pose of an intermediate link, not just the end-effector.
    step_T = smp.eye(4)
    cum_T = []
    for joint in T:
        step_T = step_T * joint
        cum_T.append(step_T)
    return cum_T


