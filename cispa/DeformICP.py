import numpy as np
from .IterClosestPoint import ICP

class DeformICP(object):
    def __init__(self, surface_mesh, mode, dk, threshold, max_iter=100) -> None:
        self.Nface = surface_mesh['Nface']
        self.vertex = surface_mesh['vertex']
        self.face_idx = surface_mesh['face_idx']
        self.mesh = surface_mesh
        self.mode = mode
        self.tip_cloud = dk
        self.threshold = threshold
        self.max_iter = max_iter
        
    def compute_transform(self, source_point, target, max_iter=100, tol=1e-6):
        """
        Find the closest point in the given mesh to the given source_point by calling ICP algorithm
        Param:
        -----------------------------------
        source: np.array, shape (N,3), the source point cloud
        target: np.array, shape (N,3), the target point cloud

        Return:
        -----------------------------------
        F: np.array, shape (4,4), the transformation matrix from source to target
        """
        ICP_ = ICP(self.mesh, threshold = [0.01, 0.01, 0.01], max_iter=100)
        F,ck = ICP_.compute_icp_transform(self.tip_cloud, search_method="Octree")
        return F, ck


if __name__ == "__main__":
    A = np.array([[1,2,3],[4,5,6],[7,8,9]])
    B = np.array([[1,2,3],[4,5,6],[7,8,9]])
    C = np.array([[1,2,3],[4,5,6],[7,8,9]])

    DICP_ = DeformICP(A, B, [0.01,0.01,0.01], 0.1)