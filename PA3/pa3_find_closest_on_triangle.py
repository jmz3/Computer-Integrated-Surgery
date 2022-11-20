import os
import sys
sys.path.append(os.path.dirname(sys.path[0]))
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cispa.FindClosestPoint2Triangle as fcp

def main():
    P = np.array([1, 1, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    A = fcp.FindClosestPoint2Triangle(P, Q.T).reshape(3)

    radius = np.linalg.norm(P-A)
    print(radius)

    fig = plt.figure(figsize=(8,8))
    plt.set_loglevel('warning')
    ax = fig.add_subplot(projection='3d')
    Q = np.vstack((Q,Q[0,:]))
    ax.plot(Q[:,0],Q[:,1],Q[:,2],marker='.')
    ax.scatter(A[0],A[1],A[2],marker='o',color='r')
    ax.scatter(P[0],P[1],P[2],marker='o',color='r')
    offset = 0.025
    ax.text(A[0]+offset,A[1]+offset,A[2]+offset,"closest point")
    ax.text(P[0]+offset,P[1]+offset,P[2]+offset,"tip point")

    u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
    x = radius * np.cos(u) * np.sin(v) + P[0]
    y = radius * np.sin(u) * np.sin(v) + P[1]
    z = radius * np.cos(v) + P[2]
    ax.plot_surface(x, y, z,alpha=0.2)

    ax.set_xlim(0.0,2.0)
    ax.set_ylim(0.0,2.0)
    ax.set_zlim(-1.0,1.0)
    plt.show()

if __name__ == "__main__":
    main()