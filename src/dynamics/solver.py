# Reference: Spong, Hutchinson & Vidyasagar, Robot Modeling and Control, Ch. 7
# Reference: SciPy solve_ivp — https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html
# Reference: SymPy lambdify — https://docs.sympy.org/latest/modules/utilities/lambdify.html

import sympy as smp
import numpy as np


def build_numerical_funcs(M, C, g, theta_syms, theta_dot_syms):
    # Converts symbolic matrices into fast NumPy-backed callables via lambdify.
    # M and g depend only on joint positions; C depends on positions and velocities.
    # Wrapping the symbol lists in an outer list tells lambdify to accept a single
    # array argument rather than one argument per symbol.
    M_func = smp.lambdify([theta_syms], M)
    C_func = smp.lambdify([theta_syms, theta_dot_syms], C)
    g_func = smp.lambdify([theta_syms], g)
    return M_func, C_func, g_func


def state_derivative(numerical_funcs, state, tau):
    # Computes the time derivative of the full state vector [theta, theta_dot].
    # Used as the right-hand side function passed to solve_ivp.
    # Rearranges the equation of motion M*theta_ddot = tau - C*theta_dot - g
    # and solves for theta_ddot with numpy.linalg.solve.
    M_func, C_func, g_func = numerical_funcs

    n = len(state) // 2
    theta = state[:n]
    theta_dot = state[n:]

    # lambdify on a SymPy Matrix returns a 2-D (n,1) array; flatten to (n,)
    # so it broadcasts correctly in the equation of motion.
    g_vec = g_func(theta).flatten()

    theta_ddot = np.linalg.solve(
        M_func(theta),
        tau - np.matmul(C_func(theta, theta_dot), theta_dot) - g_vec
    )

    return np.concatenate([theta_dot, theta_ddot])
