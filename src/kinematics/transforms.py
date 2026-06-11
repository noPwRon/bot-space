import sympy as smp

# Reference: Craig, Introduction to Robotics, Ch. 3
# Reference: CMU 16-311 Lecture 16b — https://www.cs.cmu.edu/~16311/f05/lecture/lec16b.html


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
