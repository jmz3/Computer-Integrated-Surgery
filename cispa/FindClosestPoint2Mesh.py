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

    def BruteForceSolver(self, a):
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
        if a.size != 3:
            raise ValueError("Input must be a (3,) numpy array")
    
        a.reshape(3)
        min_dist = np.inf  # Initialize the minimum distance to infinity
        for i in range(self.Nf):
            # Get the vertices of the triangle from the face index
            # p q r should be 3x1 numpy array
            p = self.vertex[self.face_idx[i, 0], :].reshape(3)
            q = self.vertex[self.face_idx[i, 1], :].reshape(3)
            r = self.vertex[self.face_idx[i, 2], :].reshape(3)

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

        h = (p + lam * (q - p) + mu * (r - p)).reshape(3)

        if lam < 0:
            h = self.ProjectOnSegment(h, r, p)
        elif mu < 0:
            h = self.ProjectOnSegment(h, p, q)
        elif lam + mu > 1:
            h = self.ProjectOnSegment(h, q, r)

        return h.reshape(3)
    
    def ProjectOnSegment(self,c,p,q):
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
        if c.shape == (3,) and p.shape == (3,) and q.shape == (3,): # Sanity check: Input must be a 3x1 numpy array
            lam = np.inner(c-p,q-p)/np.inner(q-p,q-p)
    
        else :
            raise ValueError("Input must be a numpy array with 3 elements")
        
        lam = max(0,min(1,lam))
        return p + lam * (q-p) # return the projected point
    
    def FindBoundingSphereForMesh(self):
        """
        Find the bounding sphere of the mesh
        Param:
        ---------------------------------------------------------------------------
            self: a Mesh object that contains the vertices and faces index of the mesh
        Return:
        ---------------------------------------------------------------------------
            center: 3x1 numpy array, the center of the bounding sphere
            radius: float, the radius of the bounding sphere
        """
        # Initialize the bounding sphere


        return nSphere
    
    def OctreeSolver(self, a):
        """
        Find the closest point on the mesh to the given point P using Octree
        Param:
        ---------------------------------------------------------------------------
            point: 3x1 numpy array, the point we use to find the closest point on the mesh
            self: a Mesh object that contains the vertices and faces index of the mesh

        Return:
        ---------------------------------------------------------------------------
            P_closest: 3x1 numpy array, the closest point on the mesh to P
        """
        # Size check
        if a.size != 3:
            raise ValueError("Input must be a (3,) numpy array")
    
        


if __name__ == "__main__":
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])

    # print(np.inner(P.reshape(3),P.reshape(3)))
    c_ = FindClosestPoint2Mesh(Q, 1, np.array([[0, 1, 2]]))
    print(c_.BoundingSphere(np.array([0, 1, 2])))
