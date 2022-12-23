import numpy as np
from .IterClosestPoint import ICP
from .CarteFrame import CarteFrame
from .FindClosestPoint2Triangle import FindClosestPoint2Triangle

class DeformICP(object):
    def __init__(self, surface_mesh, mode, dk, threshold, max_iter=100) -> None:
        self.Nface = surface_mesh['Nface']
        self.vertex = surface_mesh['vertex']
        self.face_idx = surface_mesh['face_idx']
        self.mesh = surface_mesh
        self.mode = mode
        self.tip_cloud = np.concatenate(dk, axis = 1).T
        self.threshold = threshold
        self.max_iter = max_iter
        self.dk = dk # List of dk
        self.sk = dk # List of sk
        self.closest_points = None
        self.Freg = CarteFrame()
        
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
        self.Freg,self.closest_points,triangle_idx = ICP_.compute_icp_transform(self.tip_cloud, search_method="Octree")
        # print("find closest points")
        sk_temp = []
        for point in self.dk:
            point = np.reshape(point, (1,3))
            sk_temp.append(self.Freg @ point)
        # print("update sk")
        self.sk = np.asarray(sk_temp).reshape(-1,3)

        return triangle_idx
    

    def compute_barycentric_coord(self):
        """
        Compute the barycentric coordinates for the given source point
        q_k = zeta_k * m_sk + xi_k * m_tk + eta_k * m_uk
        Param:
        -----------------------------------
        source_point: np.array, shape (1,3), the source point

        Return:
        -----------------------------------
        mode_combination: np.array, shape (N,1), the barycentric coordinates
        """
        barycentric_coord = np.zeros((self.Nface,1))
        triangle_idx = self.update_transform()
        bary_coord = lambda A, B: np.linalg.lstsq(A, B, rcond=None)[0] 
        mode_combination = []
        for i in range(len(triangle_idx)):
            triangle_coord = self.mesh['vertex'][self.face_idx[triangle_idx[i],:],:]
            triangle_coord = triangle_coord.T

            p = triangle_coord[:,0]
            q = triangle_coord[:,1]
            r = triangle_coord[:,2]
            A = np.concatenate((np.reshape((q-p),(3,1)), np.reshape((r-p),(3,1))),axis=1)
            # print(A.shape)
            B = np.reshape(self.closest_points[i],(3,1)) - np.reshape(p,(3,1))

            zeta = 1.0 - bary_coord(A, B)[0] - bary_coord(A, B)[1]
            xi = bary_coord(A, B)[0]
            eta = bary_coord(A, B)[1] 

            point = []
            for j in range(len(self.mode)):
                point.append(zeta*self.mode[j][self.face_idx[triangle_idx[i],0]] + \
                            xi*self.mode[j][self.face_idx[triangle_idx[i],1]] + \
                            eta*self.mode[j][self.face_idx[triangle_idx[i],2]])
            
            
            mode_combination.append(point)
        return mode_combination
    
    def compute_mode_coeff(self):
        """
        Compute the mode coefficients for the given mode
        mode_coeff = [l1,l2,l3,...,ln]
        using least square to solve A * mode_coeff = b
        where 
        A = [
            ...
            q1k, q2k, q3k, ..., qnk
            ...
        ]
        b = [ ...
            ck - q0k
            ...
        ]

        Param:
        -----------------------------------
        mode: np.array, shape (N,3), the mode to be computed

        Return:
        -----------------------------------
        mode_coeff: np.array, shape (N,1), the mode coefficients
        """
        # Compute the barycentric combination of the mode at the given source point
        # And convert the list to numpy array as size (N,7,3)
        # N is the number of source points
        # 7 is the number of modes, the first one is the origin shape
        # 3 is the dimension of each points/modes
        mode_combination = np.asarray(self.compute_barycentric_coord(), dtype=np.float32)
        A = np.zeros((3*mode_combination.shape[0],6))
        b = np.zeros((3*mode_combination.shape[0],1))
        for i in range(mode_combination.shape[0]):
            A[3*i:3*(i+1), :] = mode_combination[i,1:,:].reshape(3,-1)
            b[3*i:3*(i+1), :] = np.reshape((self.sk[i] - mode_combination[i,0,:]),(3,1))
        
        A = np.asarray(A, dtype=np.float32)
        b = np.asarray(b, dtype=np.float32)
        print(A.shape)
        print(b.shape)
        # print(np.asarray(A).shape)
        # print(np.asarray(b).shape)
        mode_coeff = np.linalg.lstsq(A, b, rcond=None)[0]
        return mode_coeff
    
    def update_mesh(self):
        for i in range(6):
        # This module need to be revised 
        # to check the convergence of the algorithm
        # and find out the correct way to update the mesh
        # range(6) is just a hardcoded number
            lam = self.compute_mode_coeff()
        
            vertices = self.mesh['vertex']
            for i in range(len(self.mode) - 1):
                vertices += -lam[i] * self.mode[i+1]
        
            self.mesh['vertex'] = vertices

            print('----------------------------------')
            print('----------Updating Mesh-----------')
            print('----------------------------------')
            # print(lam)
        
        return lam, self.Freg
    


if __name__ == "__main__":
    A = np.array([[1,2,3],[4,5,6],[7,8,9]])
    B = np.array([[1,2,3],[4,5,6],[7,8,9]])
    C = np.array([[1,2,3],[4,5,6],[7,8,9]])

    DICP_ = DeformICP(A, B, [0.01,0.01,0.01], 0.1)