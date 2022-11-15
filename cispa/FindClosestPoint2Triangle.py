import numpy as np

def FindClosestPoint2Triangle(a, vertices):
    """
    Find the closest point on the triangle to the given point P.
    Param:
    ---------------------------------------------------------------------------
        a: 3x1 numpy array, the point to be projected
        vertices: 3x3 numpy array, the vertices of the triangle, each row is a vertex

    Return:
    ---------------------------------------------------------------------------
        P_closest: 3x1 numpy array, the closest point on the mesh to P
    """

    # Sanity check: Input triangle must have 3 different
    p = vertices[0,:]
    q = vertices[1,:]
    r = vertices[2,:]
    A = np.concatenate([(p-q).reshape(3,1),(p-r).reshape(3,1)], axis=1)
    B = (a - p).reshape(3,1)

    lam,mu = np.linalg.lstsq(A,B,rcond=None)[0]

    h = p + lam * ( q - p ) + mu * ( r - p )
    if lam < 0:
        h = ProjectOnSegment(h,r,p)

    return h

def ProjectOnSegment(c,p,q):
    """
    Project the point c on the line segment pq if c is out of the triangle
    Param:
    ---------------------------------------------------------------------------
        c: 3x1 numpy array, the point to be projected
        p,q: 3x1 numpy array, the endpoints of the line segment
    
    Return:
    ---------------------------------------------------------------------------
        c_star: 3x1 numpy array, the projected point
    """
    if c.shape == (3,1):

        

if __name__=="__main__":
    P = np.array([.25, .25, .25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    diff = Q[1,:] - Q[2,:]
    A,B = FindClosestPoint2Triangle(P, Q)
    print(A)
    print(B)
