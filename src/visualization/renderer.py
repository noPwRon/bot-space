# Reference: Matplotlib 3D axes — https://matplotlib.org/stable/tutorials/toolkits/mplot3d.html
# Reference: FuncAnimation — https://matplotlib.org/stable/api/_as_gen/matplotlib.animation.FuncAnimation.html

from src.kinematics.transforms import build_T_matrices, build_cumulative_transforms
from src.kinematics.dh_table import load_robot
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import copy


def get_joint_positions(T_mat):
    """Returns a list of [x, y, z] positions, one per joint, extracted from T matrices."""
    # build_cumulative_transforms multiplies T matrices left-to-right so each position
    # is in the base frame, not relative to the previous joint.
    T_cum = build_cumulative_transforms(T_mat)

    joint_positions = []
    for T in T_cum:
        joint_positions.append(T[:3, 3])

    return joint_positions


def build_animation(robot_name, t, y):
    """
    Build a 3D animation of the robot from simulation output.

    Parameters
    ----------
    robot_name : str   — YAML filename in configs/robots/
    t          : array — 1-D time array from run_simulation
    y          : array — 2-D state array, shape [2*n_joints, len(t)]

    Returns
    -------
    FuncAnimation object (call plt.show() to display it)
    """

    robot = load_robot(robot_name=robot_name)
    n = y.shape[0] // 2

    robot_fig = plt.figure()
    ax = robot_fig.add_subplot(projection='3d')
    ax.set_xlabel("X pos [m]")
    ax.set_ylabel("Y pos [m]")
    ax.set_zlabel("Z pos [m]")
    ax.set_title(f'{n}-DoF Robot')

    initial_thetas = [joint["theta"] for joint in robot["joints"]]
    initial_ds = [joint["d"] for joint in robot["joints"]]
    modified_joint_dh = copy.deepcopy(robot["joints"])

    # Create the line object once; robot_update mutates its data each frame.
    line, = ax.plot([], [], [])

    def robot_update(frame):
        angles = y[:n, frame]
        for a, angle in enumerate(angles):
            if modified_joint_dh[a]["type"] == "revolute":
                modified_joint_dh[a]["theta"] = angle + initial_thetas[a]
            elif modified_joint_dh[a]["type"] == "prismatic":
                modified_joint_dh[a]["d"] = angle + initial_ds[a]
            else:
                raise RuntimeError(f"Unknown joint type: {modified_joint_dh[a]['type']}")

        T_matrices = build_T_matrices(modified_joint_dh)
        joint_positions = get_joint_positions(T_matrices)

        # Prepend the base origin [0, 0, 0] so the first link is drawn from the base.
        xs = [pos[0] for pos in joint_positions]
        ys = [pos[1] for pos in joint_positions]
        zs = [pos[2] for pos in joint_positions]
        xs.insert(0, 0)
        ys.insert(0, 0)
        zs.insert(0, 0)

        line.set_data(xs, ys)
        line.set_data_3d(xs, ys, zs)
        return line,

    robot_anim = FuncAnimation(robot_fig, robot_update, frames=len(t), interval=50)

    return robot_anim
