import numpy as np


class FindClosestPoint2Mesh:
    """
    Find the closest point on the triangle to the given point P.
    Param:
    ---------------------------------------------------------------------------
    a: 3x1 numpy array, the point to be projected
    vertex: Nx3 numpy array, the vertices of the triangle, each column is a vertex
    Nfaces: the number of the triangles in the mesh
    face_idx: the index of the vertices of the triangles in the mesh

    Return:
    ---------------------------------------------------------------------------
        P_closest: 3x1 numpy array, the closest point on the mesh to P
    """

    def __init__(self, vertex, Nfaces, face_idx):
        self.vertex = vertex
        self.Nf = Nfaces
        self.face_idx = face_idx

    def BruteForceSolve(self, a):
        """
        Find the closest point on the mesh to the given point P.
        Param:
        ---------------------------------------------------------------------------
            point: 3x1 numpy array, the point we use to find the closest point on the mesh
            self: a Mesh object that contains the vertices and faces index of the mesh

        Return:
        ---------------------------------------------------------------------------
            P_closest: 3x1 numpy array, the closest point on the mesh to P
        """
        # Size check
        if a.shape != (3,1):
            raise ValueError("Input must be a 3x1 numpy array")
    
        min_dist = np.inf  # Initialize the minimum distance to infinity
        for i in range(self.Nf):
            # Get the vertices of the triangle from the face index
            # p q r should be 3x1 numpy array
            p = self.vertex[self.face_idx[i, 0], :].T
            q = self.vertex[self.face_idx[i, 1], :].T
            r = self.vertex[self.face_idx[i, 2], :].T
            # Find the closest point on the triangle to the given point
            h = self.FindClosestPoint2Triangle(a, p, q, r)
            # Update the minimum distance
            if np.linalg.norm(h-a) < min_dist:
                min_dist = np.linalg.norm(h-a)
                P_closest = h

        return P_closest

    def FindClosestPoint2Triangle(self, a, p, q, r):
        A = np.concatenate([(q-p).reshape(3, 1), (r-p).reshape(3, 1)], axis=1)
        B = (a - p).reshape(3, 1)

        if A.shape == (3,2) and B.shape == (3,1): # Sanity check: Input must be 3x2 and 3x1 numpy array
            lam, mu = np.linalg.lstsq(A, B, rcond=None)[0]
        else:
            raise ValueError("A and B must be a 3x2 and 3x1 numpy array, Please check the input dimension")

        h = p + lam * (q - p) + mu * (r - p)

        if lam < 0:
            h = self.ProjectOnSegment(h, r, p)
        elif mu < 0:
            h = self.ProjectOnSegment(h, p, q)
        elif lam + mu > 1:
            h = self.ProjectOnSegment(h, q, r)

        return h.reshape(3, 1)
    
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


if __name__ == "__main__":
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
