import sympy as smp

# Reference: Lynch & Park, Modern Robotics, Ch. 4
# Reference: Craig, Introduction to Robotics, Ch. 3.5


def build_big_T(T):
    # Returns ^0T_n by sequentially multiplying all joint T matrices left to right.
    # Result[:3, 3] is end-effector position; result[:3, :3] is end-effector orientation.
    big_T = smp.eye(4)
    for joint in T:
        big_T = big_T * joint
    return big_T
