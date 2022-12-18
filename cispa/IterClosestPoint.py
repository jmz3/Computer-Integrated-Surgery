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
    
    def correspond_points(self, source_point, bound, search_method='BruteForce'):
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
        
        # Using Brute Force to find the closest point
        if search_method == 'BruteForce':
            closest_points, min_dist = self.fcp_.BruteForceSolver(source_point)
            closest_points = np.reshape(closest_points, (1,3))
        
        elif search_method == 'Octree':
            closest_points, min_dist = self.fcp_.OctreeSolver(source_point)
            closest_points = np.reshape(closest_points, (1,3))

        return closest_points, min_dist


    def compute_icp_transform(self, source_cloud, search_method='BruteForce'):
        """
        Compute the transformation matrix Freg
        Param:
        ---------------------------------------------------------------------------
            source_cloud: (N,3) numpy array, the point set of the source point cloud
            search_method: string, the method used to find the closest point in the surface mesh
    
        Return:
        ---------------------------------------------------------------------------
            Freg: 4x4 Homogeneous Transformation Matrix, Current estimated transformation
        """
        Freg = CarteFrame() # Initial transformation as identity matrix
        Ftemp = CarteFrame() # Temporary transformation as identity matrix
        eta = 100.0 # Initial threshold for distance
        iter = 0
        eps_bar_sequence = []
        convergence_flag_container = []

        while (iter<self.max_iter):
            temp_cloud = []
            A = []
            B = []
            error, error_sum, eps_max, eps_bar = 0.0 , 0.0, 0.0, 0.0
            
            # print(Freg.R)
            # print(Freg.p)
            # Step 1 transverse the source point cloud and find the corresponding points in the target mesh
            for point in source_cloud:


                #  Have to find the closest ahead in the first iteration
                point = np.reshape(point, (1,3))
                transformed_point = (Freg @ point).reshape(1,3)
                bnd = 10000.0 if iter == 0 else np.linalg.norm( transformed_point - closest_point)

                closest_point, min_distance,idx = self.correspond_points(transformed_point, bnd, search_method)
                if min_distance < eta:
                    A.append(point)
                    B.append(closest_point)
            

            # Step 2 compute the iterative transformation matrix Freg
            A = np.asarray(A).reshape(-1,3)
            B = np.asarray(B).reshape(-1,3)
            print(A.shape)

            transformation = regist_matched_points(A, B)
            Freg.R = transformation[0:3,0:3]
            Freg.p = transformation[0:3,3].reshape(3,1)

            for i in range(len(A)):
                A_temp = np.reshape(A[i,:], (1,3))
                B_temp = np.reshape(B[i,:], (1,3))
                A_transformed = np.reshape(Freg @ A_temp, (1,3))
                
                error = np.inner(A_transformed - B_temp, A_transformed - B_temp)
                error_sum += error
                eps_bar += np.sqrt(error)[0,0]
                
                if np.sqrt(error) > eps_max:
                    eps_max = (np.sqrt(error))[0,0]
            sigma = (np.sqrt(error)/len(A))[0,0]
            eps_bar = eps_bar/len(A)
            eps_bar_sequence.append(eps_bar)

            if iter > 1:
                convergence_flag = True if eps_bar_sequence[-1] / eps_bar_sequence[-2] >= 0.95 else False
            else:
                convergence_flag = False
            convergence_flag_container.append(convergence_flag)
            print("-----------------------------")
            print("Iteration: ", iter, "\nMax Error: ", eps_max, "\nSigma: ", sigma, "\nEps_bar: ", eps_bar)
            print("Convergence Flag: ", convergence_flag)
            
            # Step 3 update threshold
            eta = 3 * eps_bar
            
            # Step 4 check the termination condition
            if sigma < self.threshold[0] and (eps_bar < self.threshold[1] or eps_max < self.threshold[2] ):
                print("\nTermination Condition Reached!")
                print("Checking Convergence...")
                if convergence_flag:
                    print("Converged!")
                    break
            
            elif iter > 2 \
                and convergence_flag_container[-1] \
                and convergence_flag_container[-2] \
                and convergence_flag_container[-3]:
                print("It converges but the termination condition is not reached!")
                break
            
            iter += 1
        
        ck = []
        triangle_idx = []
        for point in source_cloud:
            point = np.reshape(point, (1,3))
            transformed_point = (Freg @ point).reshape(1,3)

            closest_point, min_distance, idx = self.correspond_points(transformed_point, 10000.0, "Octree")
            ck.append(closest_point)
            triangle_idx.append(idx)

        ck = np.asarray(ck).reshape(-1,3)
        return Freg, ck


if __name__=="__main__":

    Freg = CarteFrame()
    print(Freg.R)
