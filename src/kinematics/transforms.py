import numpy as np

# Transformation matrix for a single joint
def dh_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta),0,a],
        [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.cos(alpha),-np.sin(alpha),-d*np.sin(alpha)],
        [np.sin(theta)*np.sin(alpha),np.cos(theta)*np.sin(alpha),np.cos(alpha), d*np.cos(alpha)],
        [0,0,0,1]
    ])

# building a list of matricies
def build_T_matrices(data):
    T = []
    for joint in data["joints"]:
        T.append(dh_matrix(joint["a"],joint["alpha"],joint["d"],joint["theta"]))
    return T





