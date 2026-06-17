# Reference: SciPy solve_ivp — https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html
# Reference: Craig, "Introduction to Robotics", Ch. 6 (equations of motion)

from scipy.integrate import solve_ivp
import numpy as np
from src.controller.controller import build_simulation
from src.dynamics.solver import build_numerical_funcs, state_derivative


def make_rhs(numerical_funcs, tau):
    # Closure so solve_ivp can call rhs(t, state) without knowing about numerical_funcs or tau.
    def rhs(t, state):
        return state_derivative(numerical_funcs, state, tau)
    return rhs


def run_simulation(robot_name, initial_angles, initial_velocities, t_span, tau, t_eval=None):
    """
    Run a full forward-dynamics simulation of a robot.

    Parameters
    ----------
    robot_name         : str   — YAML filename in configs/robots/
    initial_angles     : list  — starting joint angles (radians), one per joint
    initial_velocities : list  — starting joint velocities, one per joint
    t_span             : tuple — (t_start, t_end) in seconds
    tau                : array — constant joint torques, one per joint
    t_eval             : list  — optional time points at which to record output

    Returns
    -------
    t : 1-D array of time points
    y : 2-D array of shape [2*n_joints, len(t)] — positions then velocities
    """

    # Build symbolic M, C, g matrices and the joint symbol lists.
    M, V, C, g, theta_syms, theta_dot_syms = build_simulation(robot_name)

    # Lambdify converts SymPy expressions into fast NumPy-callable functions.
    M_Func, C_Func, g_Func = build_numerical_funcs(M, C, g, theta_syms, theta_dot_syms)
    numerical_funcs = [M_Func, C_Func, g_Func]

    # solve_ivp requires one flat state vector: [theta_1...theta_n, dtheta_1...dtheta_n].
    initial_values = np.concatenate([initial_angles, initial_velocities])

    robo_rhs = make_rhs(numerical_funcs, tau)

    robot_results = solve_ivp(robo_rhs, t_span, initial_values, t_eval=t_eval)

    if not robot_results.success:
        raise RuntimeError(robot_results.message)

    return robot_results.t, robot_results.y
