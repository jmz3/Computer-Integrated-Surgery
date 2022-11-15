import numpy as np

def FindClosestPoint2Triangle(a, vertices):
    """
    Find the closest point on the triangle to the given point P.
    Param:
    ---------------------------------------------------------------------------
        a: 3x1 numpy array, the point to be projected
        vertices: 3x3 numpy array, the vertices of the triangle, each column is a vertex

    Return:
    ---------------------------------------------------------------------------
        P_closest: 3x1 numpy array, the closest point on the mesh to P
    """
    p = vertices[0,:]
    q = vertices[1,:]
    r = vertices[2,:]

    A = np.concatenate([(q-p).reshape(3,1),(r-p).reshape(3,1)], axis=1)
    B = (a - p).reshape(3,1)

    lam,mu = np.linalg.lstsq(A,B,rcond=None)[0]

    h = p + lam * ( q - p ) + mu * ( r - p )
    
    if lam < 0:
        h = ProjectOnSegment(h,r,p)
    elif mu < 0:
        h = ProjectOnSegment(h,p,q)
    elif lam + mu > 1:
        h = ProjectOnSegment(h,q,r)
    
    return h.reshape(3,1)

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
    if c.size == 3: # Sanity check: Input must be a 3x1 numpy array
        lam = np.inner(c-p,q-p)/np.inner(q-p,q-p)
    
    else :
        raise ValueError("Input must be a numpy array with 3 elements")
    
    lam = max(0,min(1,lam))
    return p + lam * (q-p) # return the projected point
        

if __name__=="__main__":
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    diff = Q[1,:] - Q[2,:]
    A = FindClosestPoint2Triangle(P, Q)
    print(A)
