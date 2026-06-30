import sympy as smp
from src.geometry.inertia_tensor import get_center_of_mass

# Reference: Spong, Robot Modeling and Control, Ch. 4.10
# Reference: Lynch & Park, Modern Robotics, Ch. 5.1


def build_jacobian_column_revolute(T_end, T_prev):
    # Returns a single 6x1 column of the Jacobian for a revolute joint.
    # T_prev is the cumulative transform to the previous joint (gives z-axis and origin).
    # T_end is a 4x4 matrix whose position column [:3, 3] is the target point (e.g. CoM).
    z = T_prev[:3, 2]
    p_end = T_end[:3, 3]
    p_prev = T_prev[:3, 3]
    linear = z.cross(p_end - p_prev)
    angular = z
    return linear.col_join(angular)


def build_jacobian_column_prismatic(T_prev):
    # Returns a single 6x1 column of the Jacobian for a prismatic joint.
    # Prismatic joints slide along z_{j-1} so there is no angular contribution.
    z = T_prev[:3, 2]
    linear = z
    angular = smp.zeros(3, 1)
    return linear.col_join(angular)


def build_link_jacobian(T_list, joints, link_index):
    # Builds the full 6×n Jacobian for link `link_index`, with the CoM as the reference point.
    # Called once per link by lagrangian.py to build the mass matrix.
    # T_list is the output of build_cumulative_transforms() from forward_kinematics.py.
    # joints is data["joints"] from the YAML loader.

    jacob_stack = []

    # Get CoM in local frame, then transform it into the base frame via the cumulative transform.
    # col_join appends a 1 to make the vector homogeneous (required for 4x4 matrix multiplication).
    l_com = get_center_of_mass(joints[link_index])
    p_com = T_list[link_index] * l_com.col_join(smp.Matrix([1]))

    for j, joint in enumerate(joints):
        # For joint 0 there is no previous transform, so use the identity (base frame).
        if j == 0:
            T_prev = smp.eye(4)
        else:
            T_prev = T_list[j-1]

        if j <= link_index:
            if joint["type"] == "revolute":
                # jacobian_column_revolute reads the target position from T_end[:3, 3].
                # Copy the cumulative transform and substitute p_com as the position column.
                T_end = T_list[link_index].copy()
                T_end[:3,3] = p_com[:3,:]
                jacob_stack.append(build_jacobian_column_revolute(T_end, T_prev))
            elif joint["type"] == "prismatic":
                jacob_stack.append(build_jacobian_column_prismatic(T_prev))
        else:
            # Joints beyond link_index cannot affect this link — column is zero.
            jacob_stack.append(smp.zeros(6, 1))

    # hstack joins a list of 6x1 columns into a single 6×n matrix.
    return smp.Matrix.hstack(*jacob_stack)


def build_end_effector_jacobian(T_cum, joints):
    end_effector_jacobian = build_link_jacobian(T_cum, joints, len(joints)-1)
    return end_effector_jacobian
