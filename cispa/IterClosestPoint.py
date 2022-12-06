import numpy as np
from .FindClosestPoint2Mesh import FindClosestPoint2Mesh
from .CarteFrame import CarteFrame

class ICP(object):
    """
    Iterative Closest Point Algorithm
    Param:
    ---------------------------------------------------------------------------
        None
    
    Return:
    ---------------------------------------------------------------------------
        Freg: 4x4 Homogeneous Transformation Matrix, Current estimated transformation
        ita: float, Current match distance threshold
    """

    def __init__(self, threshold, max_iter=100):
        self.threshold = threshold
        self.max_iter = max_iter
    
    def correspond_points(self, source_cloud, mesh):
        """
        Find the closest point in Q to each point in P
        Param:
        ---------------------------------------------------------------------------
            P: (N,3) numpy array, the point set to find the closest point
            Q: (M,3) numpy array, the surface mesh used to find closest points
        Return:
        ---------------------------------------------------------------------------
            closest_points: (N,3) numpy array, the closest points in Q to each point in P
        """





    def compute_icp_transform(self, source_cloud, target_mesh):
        """
        Compute the transformation matrix Freg
        Param:
        ---------------------------------------------------------------------------
            source_cloud: (N,3) numpy array, the point set of the source point cloud
            target_cloud: (M,3) numpy array, the point set that derived from the surface mesh
        Return:
        ---------------------------------------------------------------------------
            Freg: 4x4 Homogeneous Transformation Matrix, Current estimated transformation
        """
        Freg = CarteFrame() # Initial transformation as identity matrix
        iter = 0

        while (iter<self.max_iter):
            for point in source_cloud:

                #  Have to find the closest ahead in the first iteration
                bnd = 10000.0 if iter ==0 else np.linalg.norm( Freg @ point - closest_points)

                closest_points = self.correspond_points(point, target_mesh, bnd)
                
            Freg = self.compute_transform(source_cloud, closest_points)
            iter += 1


if __name__=="__main__":

    Freg = CarteFrame()
    print(Freg.R)
