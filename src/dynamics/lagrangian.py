import sympy as smp
from src.kinematics.jacobian import build_link_jacobian
from src.geometry.inertia_tensor import get_inertia, get_center_of_mass

# Reference: Spong, Hutchinson & Vidyasagar, Robot Modeling and Control, Ch. 7
# Reference: Lynch & Park, Modern Robotics, Ch. 8


def build_mass_matrix(T_list, joints, theta_syms):
    # Builds the n×n symbolic mass matrix M(theta).
    # Sums each link's translational and rotational inertia contribution over all links.
    # T_list is the output of build_cumulative_transforms() from forward_kinematics.py.
    # joints is data["joints"] from the YAML loader.

    n = len(joints)
    M = smp.zeros(n, n)

    for i in range(n):
        J = build_link_jacobian(T_list, joints, i)
        m = joints[i]["mass"]
        I = get_inertia(joints[i])
        # J_v (linear rows) and J_w (angular rows) split the 6×n Jacobian into its two parts.
        J_v = J[:3, :]
        J_w = J[3:, :]
        M += m * J_v.T * J_v + J_w.T * I * J_w

    return M


def build_potential_energy(T_list, joints):
    # Computes the total symbolic potential energy V(theta).
    # V is differentiated later to get the gravity torque at each joint.
    # T_list is the output of build_cumulative_transforms() from forward_kinematics.py.
    # joints is data["joints"] from the YAML loader.

    V = 0
    # Gravity points in the -z direction in the base frame.
    g = smp.Matrix([0, 0, -9.81])

    for j, joint in enumerate(joints):
        m = joint["mass"]
        r_local = get_center_of_mass(joint)
        # col_join appends a 1 to make the vector homogeneous for 4x4 matrix multiplication.
        r_base = T_list[j] * r_local.col_join(smp.Matrix([1]))
        r_base = r_base[:3, :]
        V += m * g.dot(r_base)

    return V


def build_gravity_vector(V, theta_syms):
    # Computes the n×1 gravity torque vector g(theta) by differentiating V.
    # Each element is the torque gravity applies at that joint in the current configuration.
    # V is the output of potential_energy().

    g_list = []
    for theta in theta_syms:
        g_list.append(smp.diff(V, theta))

    return smp.Matrix(g_list)


def build_coriolis_matrix(M, theta_syms, theta_dot_syms):
    # Computes the n×n Coriolis and centripetal matrix C(theta, theta_dot).
    # Derived from M using the Christoffel symbol formula — each element C[i,j] is a
    # weighted sum over k of partial derivatives of M, scaled by joint velocity theta_dot[k].
    # M is the output of mass_matrix().

    n = len(theta_syms)
    C_ij = smp.zeros(n, n)

    for i in range(n):
        for j in range(n):
            for k in range(n):
                c_ijk = smp.Rational(1, 2) * (
                    smp.diff(M[i, j], theta_syms[k])
                    + smp.diff(M[i, k], theta_syms[j])
                    - smp.diff(M[j, k], theta_syms[i])
                )
                C_ij[i, j] += c_ijk * theta_dot_syms[k]

    return C_ij
