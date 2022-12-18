import numpy as np
from .IterClosestPoint import ICP
from .FindClosestPoint2Triangle import FindClosestPoint2Triangle

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
        
    def update_transform(self, max_iter=100, tol=1e-6):
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
        ICP_ = ICP(self.mesh, threshold = [0.01, 0.01, 0.01], max_iter=max_iter)
        F,ck,triangle_idx = ICP_.compute_icp_transform(self.tip_cloud, search_method="Octree")
        return F, ck, triangle_idx
    
    def compute_mode_coeff(self):
        """
        Compute the mode coefficients for the given mode
        Param:
        -----------------------------------
        mode: np.array, shape (N,3), the mode to be computed

        Return:
        -----------------------------------
        mode_coeff: np.array, shape (N,1), the mode coefficients
        """
        mode_coeff = np.zeros((self.Nface,1))


        np.linalg.lstsq(self.mode, self.tip_cloud, rcond=None)
        return mode_coeff

    def compute_barycentric_coord(self):
        """
        Compute the barycentric coordinates for the given source point
        q_k = zeta_k * m_sk + xi_k * m_tk + eta_k * m_uk
        Param:
        -----------------------------------
        source_point: np.array, shape (1,3), the source point

        Return:
        -----------------------------------
        barycentric_coord: np.array, shape (N,1), the barycentric coordinates
        """
        barycentric_coord = np.zeros((self.Nface,1))
        F, ck, triangle_idx = self.update_transform()
        bary_coord = lambda A, B: np.linalg.lstsq(A, B, rcond=None)[0] 
        mode_combination = []
        for i in range(len(triangle_idx)):
            triangle_coord = self.vertex[self.face_idx[triangle_idx[i],:],:]
            triangle_coord = triangle_coord.T

            p = triangle_coord[:,0]
            q = triangle_coord[:,1]
            r = triangle_coord[:,2]
            A = np.concatenate((np.reshape((q-p),(3,1)), np.reshape((r-p),(3,1))),axis=1)
            # print(A.shape)
            B = np.reshape(ck[i],(3,1)) - np.reshape(p,(3,1))

            zeta = 1.0 - bary_coord(A, B)[0] - bary_coord(A, B)[1]
            xi = bary_coord(A, B)[0]
            eta = bary_coord(A, B)[1] 

            point = []
            for j in range(len(self.mode)):
                point.append(zeta*self.mode[j][self.face_idx[triangle_idx[i],0]] + \
                            xi*self.mode[j][self.face_idx[triangle_idx[i],1]] + \
                            eta*self.mode[j][self.face_idx[triangle_idx[i],2]])
            
            
            mode_combination.append(point)
        test = np.asarray(mode_combination, dtype=np.float32)
        print(test.shape)
        return barycentric_coord
    
    


if __name__ == "__main__":
    A = np.array([[1,2,3],[4,5,6],[7,8,9]])
    B = np.array([[1,2,3],[4,5,6],[7,8,9]])
    C = np.array([[1,2,3],[4,5,6],[7,8,9]])

    DICP_ = DeformICP(A, B, [0.01,0.01,0.01], 0.1)