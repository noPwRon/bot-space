import numpy as np
import sympy as smp
import hashlib
import pickle
from pathlib import Path

from src.kinematics.dh_table import load_robot
from src.kinematics.transforms import build_T_matrices, build_cumulative_transforms, build_end_effector_T
from src.dynamics.lagrangian import build_mass_matrix, build_potential_energy, build_gravity_vector, build_coriolis_matrix
from src.kinematics.jacobian import build_end_effector_jacobian


# The controller is the entry point for the simulator.
# It loads the robot config, sets up symbolic variables, and orchestrates the dynamics pipeline.


def numbify(sym_result):
    return np.array(sym_result.evalf(), dtype=float)


def build_theta_syms(joints):
    # Creates one SymPy symbol per joint to represent its position (generalised coordinate).
    # Revolute joints are named theta_0, theta_1, ...; prismatic joints d_0, d_1, ...
    # These symbols are what SymPy will differentiate with respect to when computing dynamics.
    theta_syms = []
    for j, joint in enumerate(joints):
        if joint["type"] == "revolute":
            theta_sym = smp.Symbol(f'theta_{j}')
            theta_syms.append(theta_sym)
        if joint["type"] == "prismatic":
            d_sym = smp.Symbol(f'd_{j}')
            theta_syms.append(d_sym)
    return theta_syms


def build_theta_dot_syms(joints):
    # Creates one SymPy symbol per joint to represent its velocity (generalised velocity).
    # These are independent of the position symbols — knowing a joint's angle tells you
    # nothing about how fast it's moving, so they must be separate unknowns.
    # Named theta_dot_0, theta_dot_1, ... (or d_dot_i for prismatic joints).
    theta_dot_syms = []
    for j, joint in enumerate(joints):
        if joint["type"] == "revolute":
            theta_sym = smp.Symbol(f'theta_dot_{j}')
            theta_dot_syms.append(theta_sym)
        if joint["type"] == "prismatic":
            d_sym = smp.Symbol(f'd_dot_{j}')
            theta_dot_syms.append(d_sym)
    return theta_dot_syms


def make_forward_kin_fn(joints):
    # Returns a closure that computes the numerical end-effector T matrix for a given theta array.
    # Captures the fixed joint structure so callers only need to pass joint angles.
    def forward_kin_fn(theta):
        joint_list = build_numerical_joints(joints, theta)
        T_list = build_T_matrices(joint_list)
        return numbify(build_end_effector_T(T_list))
    return forward_kin_fn


def make_jacobian_fn(joints):
    def jacobian_fn(theta):
        joint_list = build_numerical_joints(joints, theta)
        T_list = build_T_matrices(joint_list)
        T_cum = build_cumulative_transforms(T_list)
        return numbify(build_end_effector_jacobian(T_cum, joint_list))
    return jacobian_fn


def build_symbolic_joints(joints, theta_syms):
    # Injects the position symbols into the DH parameter table.
    # Each joint dictionary is copied so the original YAML data is not mutated.
    # After this step, the joints list contains SymPy symbols where the variable
    # DH parameter was — making every downstream T matrix a symbolic expression.
    # TODO: check that len(theta_syms) == len(joints).
    # A mismatch means some joints get no symbol or an index lookup fails silently.
    # Raise a ValueError stating both lengths so the mismatch is easy to diagnose.
    joint_list = []
    for j, joint in enumerate(joints):
        if joint["type"] == "revolute":
            theta_sym = theta_syms[j]
            joint_list.append(joints[j].copy())
            joint_list[j]["theta"] += theta_sym
        if joint["type"] == "prismatic":
            d_sym = theta_syms[j]
            joint_list.append(joints[j].copy())
            joint_list[j]["d"] += d_sym
    return joint_list

def build_numerical_joints(joints, theta_nums):
    # Substitutes numerical joint angle values into the DH parameter table.
    # Each joint dictionary is copied so the original data is not mutated.
    # theta_nums is an array of values, one per joint, in the same order as joints.
    # TODO: check that len(theta_nums) == len(joints).
    # Too few values causes a silent skip; too many causes an IndexError inside the loop.
    # Raise a ValueError stating both lengths so the mismatch is easy to diagnose.
    joint_list = []
    for j, joint in enumerate(joints):
        if joint["type"] == "revolute":
            theta_num = theta_nums[j]
            joint_list.append(joints[j].copy())
            joint_list[j]["theta"] = theta_num
        if joint["type"] == "prismatic":
            d_num = theta_nums[j]
            joint_list.append(joints[j].copy())
            joint_list[j]["d"] = d_num
    return joint_list


def build_simulation(robot_name):
    # Orchestrates the full symbolic dynamics pipeline for a given robot.
    # Checks for a cached result first to avoid repeating the expensive symbolic build.
    # Returns everything the solver and simulator will need.

    # Step 1: load the robot's DH parameters and link properties from YAML.
    robot, config_path = load_robot(robot_name)

    # Step 2: hash the raw YAML file bytes to create a cache key.
    # If the robot config changes, the hash changes and the cache is ignored.
    # Use hashlib to read the YAML file as bytes and produce a hex digest string.
    # Combine the robot name and hash to form a unique cache filename, e.g.
    # "cache/example_6dof_<hash>.pkl". Use pathlib.Path to build the path.
    with open(config_path,"rb") as f:
        file_hash = hashlib.md5(f.read()).hexdigest()


    # Step 3: check whether the cache file exists.
    # pathlib.Path has a method that returns True if a file exists at that path.
    # If it exists, open it in binary read mode and use pickle.load to deserialise
    # the tuple (M, C, g, theta_syms, theta_dot_syms), then return it immediately.
    # This skips all the symbolic computation below.

    cache_dir = Path("cache")
    cache_file = cache_dir / f"{robot_name}_{file_hash}.pkl"


    # Step 4: create symbolic placeholders for joint positions and velocities.
    # These are the generalised coordinates q and q_dot in the Lagrangian formulation.
    theta_syms = build_theta_syms(robot["joints"])
    theta_dot_syms = build_theta_dot_syms(robot["joints"])

    # Step 5: substitute position symbols into the DH table so all T matrices
    # will be functions of the symbolic joint variables rather than fixed numbers.
    joints = build_symbolic_joints(robot["joints"], theta_syms)

    # Step 6: build the individual joint T matrices, then multiply them left-to-right
    # to get the cumulative transform from the base frame to each link.
    T_list = build_T_matrices(joints)
    T_cum = build_cumulative_transforms(T_list)

    # Step 7: compute the four dynamic quantities that define the equations of motion.
    # M — inertia matrix (how much each joint resists acceleration)
    M = build_mass_matrix(T_cum, joints, theta_syms)
    # V — total potential energy (used to derive gravity torques)
    V = build_potential_energy(T_cum, joints)
    # g — gravity torque vector (partial derivatives of V w.r.t. each joint)
    g = build_gravity_vector(V, theta_syms)
    # C — Coriolis/centripetal matrix (velocity-dependent torques)
    C = build_coriolis_matrix(M, theta_syms, theta_dot_syms)

    # Step 8: save the result to the cache file before returning.
    # Create the cache/ directory if it does not exist yet — pathlib.Path has a
    # method for this that accepts a flag to avoid errors if it already exists.
    # Open the cache file in binary write mode and use pickle.dump to serialise
    # the tuple (M, C, g, theta_syms, theta_dot_syms).

    # Step 9: return all symbolic quantities for use by the solver and simulator.
    return M, C, g, theta_syms, theta_dot_syms
