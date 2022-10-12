from __future__ import annotations
# from selectors import EpollSelector
from typing import Union, overload
import numpy as np
from scipy.spatial.transform import Rotation


class FrameTransform:
    @property
    def R(self) -> np.ndarray:
        return self._R

    @R.setter
    def R(self, value: np.ndarray):
        if not isinstance(value, np.ndarray):
            raise TypeError(f"R must be a numpy.ndarray, not {type(value)}")
        if value.shape != (3, 3):
            raise ValueError(f"R must be a 3x3 matrix, not {value.shape}")

        d = np.linalg.det(value)
        if not np.isclose(d, 1.0):
            raise ValueError(f"R must be a rotation matrix, det(R) = {d}")

        self._R = value

    @property
    def p(self) -> np.ndarray:
        return self._p

    @p.setter
    def p(self, value: np.ndarray):
        if not isinstance(value, np.ndarray):
            raise TypeError(f"p must be a numpy.ndarray, not {type(value)}")
        if value.shape != (3,):
            raise ValueError(f"p must be a 3x1 vector, not {value.shape}")

        self._p = value

    def __init__(self, R: np.ndarray, p: np.ndarray):
        self.R = R
        self.p = p

    def __str__(self):
        return f"R = {self.R}\np = {self.p}"

    @overload
    def __matmul__(self, other: FrameTransform) -> FrameTransform:
        pass

    @overload
    def __matmul__(self, other: np.ndarray) -> np.ndarray:
        pass

    def __matmul__(self, other):
        if isinstance(other, FrameTransform):
            pass
        elif isinstance(other, np.ndarray):
            pass
        else:
            raise TypeError(f"Can't multiply FrameTransform with {type(other)}")

    def inverse(self) -> FrameTransform:
        pass

    @classmethod
    def from_matched_points(cls, X: np.ndarray, Y: np.ndarray):
        # Your implementation
        Data_Num = np.size(X,0)

        F_d,F_0,F = initialize()

        X = np.append(X,np.ones(shape=(Data_Num,1)),1)
        Y = np.append(Y,np.ones(shape=(Data_Num,1)),1)
        X = np.transpose(X)
        Y = np.transpose(Y)

        while ( True ):
            X_k = np.matmul(F,X)

            # Least Square Terms are Ax=b
            A,b = Calc_Coefficient(X_k,Y)
            print(A.shape)
            print(b.shape)
            error = np.linalg.lstsq(A,b)

            F_d = np.hstack(skew(error[0:2]),error[3:5])
            F_d = np.vstack(F_d,np.array([0,0,0,1]))
            print(F_d)
            break
            F = F_d @ F
            # if error.norm < some_threshold : break
            
        R = np.eye(3) + skew(error[1:3])
        p = np.zeros(3)

        return cls(R, p)

def initialize():
    return np.eye(4),np.eye(4),np.eye(4)

def Calc_Coefficient(X: np.ndarray, Y: np.ndarray):
    A = np.empty((1,6))
    b = np.empty(1)

    for i in range(np.size(X,1)):

        A_i = np.hstack((skew(-X[:2,i]),np.eye(3)))
        A = np.append(A,A_i,0)

        bi = Y[2:-1,i] - X[:2,i]
        b = np.append(b,bi,0)

    A = np.delete(A,0,0) # remove the first empty row
    b = np.delete(b,0,0)
    print(b)
    return A,b

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
