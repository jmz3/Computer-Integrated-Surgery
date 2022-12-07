import numpy as np
from .FindClosestPoint2Mesh import FindClosestPoint2Mesh
from .CarteFrame import CarteFrame
from .Registration import regist_matched_points

class ICP(object):
    """
    Iterative Closest Point Algorithm

    Param:
    ---------------------------------------------------------------------------
        surface_mesh: dictionary, the surface mesh containing the following:
                        Nface: int, number of faces in the surface mesh
                        vertex: (M,3) numpy array, the point set that derived from the surface mesh
                        face_idx: (N,3) numpy array, the index of the vertex in the surface mesh
        threshold: (3,) numpy array, the threshold for the distance between the source point and the closest point in the surface mesh
        max_iter: int, the maximum number of iterations to avoid infinite loop
    
    Return:
    ---------------------------------------------------------------------------
        Freg: 4x4 Homogeneous Transformation Matrix, Current estimated transformation
        ita: float, Current match distance threshold
    """

    def __init__(self, surface_mesh : dict ,  threshold : np.array, max_iter=100):
        self.threshold = threshold
        self.max_iter = max_iter
        self.Nface = surface_mesh['Nface']
        self.vertex = surface_mesh['vertex']
        self.face_idx = surface_mesh['face_idx']
        self.fcp_ = FindClosestPoint2Mesh(self.vertex, self.Nface, self.face_idx)
    
    def correspond_points(self, source_point, bound):
        """
        Find the closest point in the given mesh to the given source_point
        Param:
        ---------------------------------------------------------------------------
            source_point: (1,3) numpy array, the point used to find the closest point
            mesh: (M,3) numpy array, the surface mesh used to find closest points
        Return:
        ---------------------------------------------------------------------------
            closest_points: (N,3) numpy array, the closest points in Q to each point in P
        """
        # size check 
        if source_point.shape != (1,3):
            raise ValueError("Input must be a (1,3) numpy array")
        closest_points, min_dist = self.fcp_.BruteForceSolver(source_point)
        closest_points = np.reshape(closest_points, (1,3))
        return closest_points, min_dist


    def compute_icp_transform(self, source_cloud):
        """
        Compute the transformation matrix Freg
        Param:
        ---------------------------------------------------------------------------
            source_cloud: (N,3) numpy array, the point set of the source point cloud
    
        Return:
        ---------------------------------------------------------------------------
            Freg: 4x4 Homogeneous Transformation Matrix, Current estimated transformation
        """
        Freg = CarteFrame() # Initial transformation as identity matrix
        ita = 100.0 # Initial threshold for distance
        iter = 0

        while (iter<self.max_iter):
            A = []
            B = []
            
            # Step 1 transverse the source point cloud and find the corresponding points in the target mesh
            for point in source_cloud:

                error,eps, eps_bar = 0.0 , 0.0, 0.0
                eps_bar_sequence = []
                #  Have to find the closest ahead in the first iteration
                point = np.reshape(point, (1,3))
                transformed_point = Freg @ point
                bnd = 10000.0 if iter ==0 else np.linalg.norm( transformed_point - closest_point)

                closest_point, min_distance = self.correspond_points(point, bnd)
                if min_distance < ita:
                    A.append(point)
                    B.append(closest_point)
            
            # Step 2 compute the iterative transformation matrix Freg
            A = np.asarray(A).reshape(-1,3)
            B = np.asarray(B).reshape(-1,3)
            print(A.shape)
            print(B.shape)
            transformation = regist_matched_points(A, B)
            Freg.R = transformation[0:3,0:3]
            Freg.p = transformation[0:3,3]
            for i in range(len(A)):
                error += np.inner(A[i] - B[i], A[i] - B[i])
                eps_bar += np.linalg.norm(A[i] - B[i])
                if np.sqrt(error) > eps:
                    eps = np.sqrt(error)
            sigma = np.sqrt(error)/len(A)
            eps_bar = eps_bar/len(A)
            eps_bar_sequence.append(eps_bar)

            print("--------------------")
            print("\nIteration: ", iter, "\nError: ", eps, "\nSigma: ", sigma, "\nEps_bar: ", eps_bar)
            
            # Step 3 update threshold
            ita = 3 * eps_bar
            
            # Step 4 check the termination condition
            if sigma < 0.001 and eps_bar < 0.001:
                print("\nTermination Condition Reached!")
                print("\nChecking Convergence...\n")
                if eps_bar_sequence[-1] / eps_bar_sequence[-2] >= 0.95:
                    break
            else:
                iter += 1
        return Freg


if __name__=="__main__":

    Freg = CarteFrame()
    print(Freg.R)
