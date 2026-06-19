from src.dynamics.lagrangian import build_mass_matrix, build_coriolis_matrix, build_potential_energy, build_gravity_vector
import numpy as np


def test_massMatrix(joint_build):

    joint, theta_syms, theta_dot_syms, T_cum = joint_build
    
    M = build_mass_matrix(T_cum,joint,theta_syms)

    M_num = np.array(M.evalf(), dtype=float)
    
    eigenvalues = np.linalg.eigvalsh(M_num)

    assert np.all(eigenvalues > 0)

def test_coriolisMatrix(joint_build):

    joint, theta_syms, theta_dot_syms, T_cum = joint_build
    
    M = build_mass_matrix(T_cum,joint,theta_syms)

    C = build_coriolis_matrix(M,theta_syms,theta_dot_syms)

    C_0 = C.subs(theta_dot_syms[0],0)

    assert np.allclose(C_0, np.array([0]))

def test_gVec(joint_build):

    joint, theta_syms, theta_dot_syms, T_cum = joint_build

    V = build_potential_energy(T_cum, joint)

    g_vec = build_gravity_vector(V,theta_syms)

    g_vec_0 = g_vec.subs(theta_syms[0],0)
    
    assert np.allclose(g_vec_0, np.array([0]))




    


