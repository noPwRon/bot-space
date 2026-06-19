import sympy as smp

from src.kinematics.dh_table import load_robot
from src.kinematics.transforms import build_T_matrices, build_cumulative_transforms, build_end_effector_T
from src.dynamics.lagrangian import build_mass_matrix, build_potential_energy, build_gravity_vector, build_coriolis_matrix

# The controller is the entry point for the simulator.
# It loads the robot config, sets up symbolic variables, and orchestrates the dynamics pipeline.


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
        return build_end_effector_T(T_list)
    return forward_kin_fn


def build_symbolic_joints(joints, theta_syms):
    # Injects the position symbols into the DH parameter table.
    # Each joint dictionary is copied so the original YAML data is not mutated.
    # After this step, the joints list contains SymPy symbols where the variable
    # DH parameter was — making every downstream T matrix a symbolic expression.
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
    # Returns everything the solver and simulator will need.

    # Step 1: load the robot's DH parameters and link properties from YAML.
    robot = load_robot(robot_name)

    # Step 2: create symbolic placeholders for joint positions and velocities.
    # These are the generalised coordinates q and q_dot in the Lagrangian formulation.
    theta_syms = build_theta_syms(robot["joints"])
    theta_dot_syms = build_theta_dot_syms(robot["joints"])

    # Step 3: substitute position symbols into the DH table so all T matrices
    # will be functions of the symbolic joint variables rather than fixed numbers.
    joints = build_symbolic_joints(robot["joints"], theta_syms)

    # Step 4: build the individual joint T matrices, then multiply them left-to-right
    # to get the cumulative transform from the base frame to each link.
    T_list = build_T_matrices(joints)
    T_cum = build_cumulative_transforms(T_list)

    # Step 5: compute the four dynamic quantities that define the equations of motion.
    # M — inertia matrix (how much each joint resists acceleration)
    M = build_mass_matrix(T_cum, joints, theta_syms)
    # V — total potential energy (used to derive gravity torques)
    V = build_potential_energy(T_cum, joints)
    # g — gravity torque vector (partial derivatives of V w.r.t. each joint)
    g = build_gravity_vector(V, theta_syms)
    # C — Coriolis/centripetal matrix (velocity-dependent torques)
    C = build_coriolis_matrix(M, theta_syms, theta_dot_syms)

    # Step 6: return all symbolic quantities for use by the solver and simulator.
    return M, C, g, theta_syms, theta_dot_syms
