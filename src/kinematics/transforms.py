import sympy as smp

# Reference: Craig, Introduction to Robotics, Ch. 3
# Reference: CMU 16-311 Lecture 16b — https://www.cs.cmu.edu/~16311/f05/lecture/lec16b.html


def dh_matrix(a, alpha, d, theta):
    # Returns the 4x4 modified DH transformation matrix ^(i-1)T_i for one joint.
    return smp.Matrix([
        [smp.cos(theta),                -smp.sin(theta),                 0,              a],
        [smp.sin(theta)*smp.cos(alpha),  smp.cos(theta)*smp.cos(alpha), -smp.sin(alpha), -d*smp.sin(alpha)],
        [smp.sin(theta)*smp.sin(alpha),  smp.cos(theta)*smp.sin(alpha),  smp.cos(alpha),  d*smp.cos(alpha)],
        [0,                              0,                               0,              1]
    ])


def build_T_matrices(data):
    # Returns a list of symbolic T matrices, one per joint, using fixed YAML values.
    # When symbolic joint variables are needed, the controller passes
    # joint["theta"] + theta_symbol (revolute) or joint["d"] + d_symbol (prismatic)
    # in place of the raw YAML value, using joint["type"] to determine which.
    T = []
    for joint in data["joints"]:
        T.append(dh_matrix(joint["a"], joint["alpha"], joint["d"], joint["theta"]))
    return T
