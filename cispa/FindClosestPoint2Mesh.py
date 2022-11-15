import numpy as np

class FindClosestPoint2Mesh:
    """
    Find the closest point on the triangle to the given point P.
    Param:
    ---------------------------------------------------------------------------
    a: 3x1 numpy array, the point to be projected
    vertex: Nx3 numpy array, the vertices of the triangle, each column is a vertex
        
    Return:
    ---------------------------------------------------------------------------
        P_closest: 3x1 numpy array, the closest point on the mesh to P
    """
    def __init__(self, vertex, face_idx):
        self.vertex = vertex
        self.face_idx = face_idx


    def FindClosestPoint2Mesh(self, a):
        """
        Find the closest point on the mesh to the given point P.
        Param:
        ---------------------------------------------------------------------------
            a: 3x1 numpy array, the point to be projected
            mesh: a Mesh object, the mesh to be projected on

        Return:
        ---------------------------------------------------------------------------
            P_closest: 3x1 numpy array, the closest point on the mesh to P
        """

if __name__=="__main__":
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])