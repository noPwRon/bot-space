import sympy as smp

# Geometric Jacobian — each function returns one 6x1 column vector [linear; angular].
# For joint 1, pass smp.eye(4) as T_prev (base frame).
# Reference: Spong, Robot Modeling and Control, Ch. 4.10
# Reference: Lynch & Park, Modern Robotics, Ch. 5.1


def jacobian_column_revolute(T_end, T_prev):
    # z_{j-1}, p_{j-1}, and p_n extracted from the cumulative transforms.
    z = T_prev[:3, 2]
    p_end = T_end[:3, 3]
    p_prev = T_prev[:3, 3]
    linear = smp.cross(z, p_end - p_prev)
    angular = z
    return smp.Matrix([linear, angular])


def jacobian_column_prismatic(T_prev):
    # Prismatic joints produce only linear velocity along z_{j-1}; no angular component.
    z = T_prev[:3, 2]
    linear = z
    angular = smp.zeros(3)
    return smp.Matrix([linear, angular])
