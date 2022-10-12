import numpy as np
from .DataProcess import centralize
from scipy.linalg import svd


def regist_matched_points(X: np.ndarray, Y: np.ndarray):

    # type check and reshape the input data as:
    # [[x1 x2 x3 ...]
    #  [y1 y2 y2 ...]
    #  [z1 z2 z3 ...]]
    if not isinstance(X, np.ndarray):
        raise TypeError(f"Input Data must be numpy array!")

    if np.size(X,1)==3:
        Data_Num = np.size(X,0)
        X = X.T
        Y = Y.T
    else:
        Data_Num = np.size(X,1)

    # centralization
    X_bar, X_mean = centralize(X)
    Y_bar, Y_mean = centralize(Y)

    # Compute Rotation
    H = np.zeros((3,3))
    for i in range(Data_Num): H = H + np.outer(X_bar[:,i],Y_bar[:,i])
    U,Sigma,V_T = svd(H)

    V = np.transpose(V_T)
    U_T = np.transpose(U)

    R = V @ np.diag([1,1,np.linalg.det(V)*np.linalg.det(U_T)]) @ U_T

    # Compute translation
    p = Y_mean - R @ X_mean

    # Concatenate F
    F = np.vstack([np.hstack([R,p]),np.array([0,0,0,1])])
    return F




