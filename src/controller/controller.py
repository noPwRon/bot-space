import sympy as smp

from src.kinematics.dh_table import load_robot
from src.kinematics.transforms import build_T_matrices
from src.kinematics.forward_kinematics import build_cumulative_transforms
from src.dynamics.lagrangian import build_mass_matrix, build_potential_energy, build_gravity_vector, build_coriolis_matrix

# The controller is the entry point for the simulator.
# It loads the robot config, sets up symbolic variables, and orchestrates the dynamics pipeline.


def build_symbolic_joints(joints, theta_syms):
    # Takes the raw joints list and a list of SymPy symbols (one per joint).
    # Returns a new joints list where each joint's variable parameter (theta for
    # revolute, d for prismatic) has been replaced with its corresponding symbol.
    # The original joints list is not modified.
    joint_list = []
    for j, joint in enumerate(joints):

        if joint["type"] == "revolute":
            theta_sym = theta_syms[j]
            joint_list.append(joints[j].copy())
            joint_list[j]["theta"] = theta_sym
        if joint["type"] == "prismatic":
            d_sym = theta_syms[j]
            joint_list.append(joints[j].copy())
            joint_list[j]["d"] = d_sym
    return joint_list
    
def build_theta_syms(joints):
    # Takes the raw joints list and returns a list of SymPy symbols, one per joint.
    # Revolute joints get a theta symbol; prismatic joints get a d symbol.
    # The order matches the joint order so index i in the result corresponds to joint i.
    theta_syms = []
    for j, joint in enumerate(joints):
        if joint["type"] == "revolute":
            theta_sym = smp.Symbol(f'theta{j}')
            theta_syms.append(theta_sym)
        if joint["type"] == "prismatic":
            d_sym = smp.Symbol(f'd{j}') 
            theta_syms.append(d_sym)
    return theta_syms

def build_simulation(robot_name):
    # Loads the robot config and builds all symbolic structures needed for dynamics.
    # Returns everything the simulation and solver will need.
    #
    # Steps:
    # 1. Load the robot YAML using load_robot.
    # 2. Create one symbolic variable per joint for position and one for velocity.
    #    These are the generalised coordinates — the theta and theta_dot vectors.
    # 3. Build the symbolic joint list by substituting those variables into the DH parameters.
    # 4. Build the individual T matrices, then the cumulative transform list.
    # 5. Compute M, V, g, and C using the lagrangian functions.
    # 6. Return all of the above for use by the solver and simulator.
    pass
