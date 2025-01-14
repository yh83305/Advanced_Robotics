import numpy as np

def space_jacobi(theta, w, v):
    n = w.shape[0]
    J = np.zeros((6, n))
    S = np.vstack((w.T, v.T))
    for i in range(n):
        if i == 0:
            J[:, i] = S[:, i]
        else:
            J[:, i] = Adjoint(sub_exp(theta, w, v, i)) @ S[:, i]
    return J
def sub_exp(theta, w, v, k):
    T = np.eye(4)
    for i in range(k):
        T = T @ exp2T(theta[i], w[i, :], v[i, :])
    return T

def Adjoint(T):
    # Extract rotation matrix R and translation vector p from T
    R = T[:3, :3]
    p = T[:3, 3]
    # Create the skew-symmetric matrix of the translation vector p
    p_skew = vector_to_skew_symmetric(p)
    # Create the adjoint representation
    adj_T = np.block([
                     [R, np.zeros((3, 3))],
                     [p_skew @ R, R]
                     ])
    return adj_T
def PoE(theta, w, v, M):
    n = w.shape[0]
    T = np.eye(4)
    for i in range(n):
        T = T @ exp2T(theta[i], w[i, :], v[i, :])
    return T @ M

def exp2T(theta, w, v):
    R = Rodrigues(w, theta)
    p = G(w, theta) @ v
    top = np.hstack((R, p.reshape(-1, 1)))
    bottom = np.hstack((np.zeros((1, 3)), np.eye(1)))
    return np.vstack((top, bottom))
def Rodrigues(w_3, theta):
    w_skew = vector_to_skew_symmetric(w_3)
    return np.eye(3) + np.sin(theta) * w_skew + (1 - np.cos(theta)) * w_skew @ w_skew

def G(w_3, theta):
    w_skew = vector_to_skew_symmetric(w_3)
    return np.eye(3) * theta + (1 - np.cos(theta)) * w_skew + (theta - np.sin(theta)) * w_skew @ w_skew

def vector_to_skew_symmetric(v_3):
    return np.array([[0, -v_3[2], v_3[1]],
                     [v_3[2], 0, -v_3[0]],
                     [-v_3[1], v_3[0], 0]])

def mergeRp(R,p):
    top = np.hstack((R, p.reshape(-1, 1)))
    bottom = np.hstack((np.zeros((1, 3)), np.eye(1)))
    return np.vstack((top, bottom))

def relative_pose(T_A, T_B):
    R_B = T_B[:3, :3]
    p_B = T_B[:3, 3]

    R_B_inv = R_B.T  # R 的转置为其逆矩阵
    p_B_inv = -R_B_inv @ p_B  # 计算逆的平移部分

    # 构造 T_B 的逆矩阵
    T_B_inv = np.eye(4)
    T_B_inv[:3, :3] = R_B_inv
    T_B_inv[:3, 3] = p_B_inv

    # 计算 T_AB = T_B_inv * T_A
    T_AB = T_B_inv @ T_A

    return T_AB