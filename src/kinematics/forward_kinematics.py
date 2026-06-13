import sympy as smp

# Reference: Lynch & Park, Modern Robotics, Ch. 4
# Reference: Craig, Introduction to Robotics, Ch. 3.5


def build_end_effector_T(T):
    # Returns ^0T_n by sequentially multiplying all joint T matrices left to right.
    # Result[:3, 3] is end-effector position; result[:3, :3] is end-effector orientation.
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

