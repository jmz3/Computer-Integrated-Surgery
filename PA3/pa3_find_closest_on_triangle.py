import numpy as np
import matplotlib.pyplot as plt
import cispa.FindClosestPoint2Triangle as FindClosestPoint2Triangle

def main():
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    A = FindClosestPoint2Triangle(P, Q.T)
    print(A)