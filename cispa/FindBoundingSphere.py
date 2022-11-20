import numpy as np

class SingleSphere(object):
    def __init__(self, vertices, triangle_idx) -> None:
        """
        Initialize the class
        Param:
        ---------------------------------------------------------------------------
            vertices: 3x3 numpy array, the vertices of the mesh
            the vertices are stacked as row vectors i.e. 
            [px py pz 
             qx qy qz 
             rx ry rz]
            face_idx: int, the index of the face_idx
        """
        self.vertices = vertices
        self.triangle_idx = triangle_idx
        self.center, self.radius = self.FindBoundingSphereForTriangle(vertices)
    
    def FindBoundingSphereForTriangle(self,vertices):
        """
        Find the bounding sphere of a triangle
        Param:
        ---------------------------------------------------------------------------
            self: a Mesh object that contains the vertices and faces index of the mesh
            triangle_idx: the index of the triangle that needs to be computed
        Return:
        ---------------------------------------------------------------------------
            center: 3x1 numpy array, the center of the bounding sphere
            radius: float, the radius of the bounding sphere
        """
        # Find the longest edge of the triangle
        p = vertices[0, :].reshape(3)
        q = vertices[1, :].reshape(3)
        r = vertices[2, :].reshape(3)
        edge_1 = np.linalg.norm(q-p)
        edge_2 = np.linalg.norm(r-q)
        edge_3 = np.linalg.norm(p-r)

        # Compute the corresponding u and v based on the choice of longest edge
        if edge_1 >= edge_2 and edge_1 >= edge_3:
            f = (p + q)/2
            u = p - f
            v = r - f
        elif edge_2 >= edge_1 and edge_2 >= edge_3:
            f = (q + r)/2
            u = q - f
            v = p - f
        elif edge_3 >= edge_1 and edge_3 >= edge_2:
            f = (r + p)/2
            u = r - f
            v = q - f
        else:
            raise ValueError("Something wrong with the triangle")

        d = np.cross(np.cross(u, v),u)
        gamma = (np.inner(v,v) - np.inner(u,u)) / (2 * np.inner(d,(v-u)))

        lam = 0 if gamma <= 0 else gamma

        # Compute the center and radius of the bounding sphere
        center = f + lam * d
        radius = np.linalg.norm(center - p)
        return center, radius
    
    def FindClosestPoint2Triangle(self, a):
        """
        Find the closest point on the triangle to the point a
        Param:  
        ---------------------------------------------------------------------------
        a: 3x1 numpy array, the point to be projected
        Return:
        ---------------------------------------------------------------------------
        c: 3x1 numpy array, the closest point on the triangle to the point a
        """
        p = self.vertices[0, :].reshape(3)
        q = self.vertices[1, :].reshape(3)
        r = self.vertices[2, :].reshape(3)

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
    
