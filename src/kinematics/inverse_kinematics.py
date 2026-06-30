# Reference: Spong, Robot Modeling and Control, Ch. 6 (Jacobian-based IK)
# Reference: scipy.optimize — https://docs.scipy.org/doc/scipy/reference/optimize.html

import numpy as np


def ik_newton_raphson(target_T, theta_init, forward_kin_fn, jacobian_fn, tol=1e-6, max_iter=100):
    theta = theta_init.copy()

    for _ in range(max_iter):
        current_T = forward_kin_fn(theta)
        error = compute_error(target_T, current_T)

        if np.linalg.norm(error) < tol:
            break

        J = jacobian_fn(theta)
        J_pinv = np.linalg.pinv(J)
        theta = theta + J_pinv @ error

    return theta


def compute_error(target_T, current_T):
    pos_error = target_T[:3, 3] - current_T[:3, 3]
    rot_error = target_T[:3, :3] - current_T[:3, :3]
    return np.concatenate([pos_error, rot_error.flatten()])
