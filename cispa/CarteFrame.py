from __future__ import annotations
import numpy as np


class CarteFrame(object):
    def __init__(self, R=np.eye(3), p=np.zeros((3, 1))) -> None:
        # Check demensionality of p
        if p.shape == (3,1):
            pass
        elif p.shape == (1,3):
            p = p.T
        else:
            try:
                p = p.reshape((3,1))
            except:
                raise Exception("Wrong size for input position vector.")

        self.R = R
        self.p = p

    def __matmul__(self, other: np.ndarray):
        if isinstance(other, np.ndarray):
            # if the input is a 3x1 position vector
            if other.shape == (3, 1):
                p = self.p + self.R @ other

                return p

            # if the input is a 1x3 vector        
            elif other.shape == (1, 3):
                other = other.T
                p = self.p + self.R @ other
                return p

            else:
                raise Exception("Wrong size for input position vector.")

        elif isinstance(other, CarteFrame):
            R = np.matmul(self.R , other.R)
            p = np.matmul(self.R, other.p) + self.p
            return CarteFrame(R, p)
    
    def inverse(self):
        R = self.R.T
        p = - self.R.T @ self.p
        self.R = R
        self.p = p


if __name__ == "__main__":
    R = np.array([[1,0,0],[0,0,1],[0,-1,0]])
    p = np.array([0,0,1])
    F = CarteFrame(R,p)
    F1 = F@F
    print(F.R)
    F.inverse()
    print(F.R)
    print(F.p)
    print(pow(2,3))

    poly = np.array([1,2,3,4,5,6])
    poly.reshape(6,1)
    T = np.zeros((6,3))

    T[:,0] = [poly[i] for i in range(6)]
    print(T)
