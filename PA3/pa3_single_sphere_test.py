import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
import cispa.FindBoundingSphere as FBS
import numpy as np

def main():
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])

    S = FBS.SingleSphere(Q, np.array([0, 1, 2]))
    print(S.center)
    print(S.radius)

if __name__=="__main__":
    main()