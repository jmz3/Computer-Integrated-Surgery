import numpy as np
from typing import overload

class HomogeneousVector:

    def __init__(self,p) -> None:
        self.p = p
    
    def Homogeneous(self):
        return np.vstack((self.p,1))

    @overload
    def __matmul__(self, other: np.ndarray) -> HomogeneousVector:
        pass