import numpy as np

class Sphere(object):
    def __init__(self, vertices, triangle_idx) -> None:
        """
        Initialize the class
        Param:
        ---------------------------------------------------------------------------
            vertices: 3x3 numpy array, the vertices of the mesh
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
        p = vertices[0:3, :].reshape(3)
        q = vertices[0:3, :].reshape(3)
        r = vertices[0:3, :].reshape(3)
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