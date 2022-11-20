import numpy as np
from .FindBoundingSphere import SingleSphere

class Octree:
    """
    Octree class for the mesh
    """
    def __init__(self, Spheres, nSpheres):
        """
        Initialize the Octree
        Param:
        ---------------------------------------------------------------------------
            mesh: a Mesh object that contains the vertices and faces index of the mesh
        Return:
        ---------------------------------------------------------------------------
            None
        """
        self.Spheres = Spheres
        self.nSpheres = nSpheres
        self.CentroidPoint = self.Centroid()
        self.MaxRadius = self.FindMaxRadius()
        self.UB = self.FindMaxCoordinates()
        self.LB = self.FindMinCoordinates()
        self.Subtrees = [[[[] for i in range(2)] for j in range(2)] for k in range(2)]
        self.ConstructSubtrees()

    # def GenerateSpheres(self):
    #     """
    #     Generate the bounding spheres for the mesh
    #     Param:
    #     ---------------------------------------------------------------------------
    #         None
    #     Return:
    #     ---------------------------------------------------------------------------
    #         Spheres: list of Sphere, the bounding spheres for the mesh
    #         nSpheres: int, the number of bounding spheres
    #     """
    #     Spheres = []
    #     for i in range(self.Nfaces):
    #         face_idx = self.face_idx[i]
    #         v1 = self.vertex[face_idx[0]]
    #         v2 = self.vertex[face_idx[1]]
    #         v3 = self.vertex[face_idx[2]]
    #         v = np.vstack([v1, v2, v3])
    #         Sphere = SingleSphere(v,face_idx)
    #         Spheres.append(Sphere)

    #     return Spheres

    def ConstructSubtrees(self):
        # Stop Critiria:
        # Tree stops to grow when the number of spheres in the tree is less than 2

        if self.nSpheres < 2:
            self.HaveSubtrees = False
            return
        self.HaveSubtrees = True
        # print(self.nSpheres)
        SubSpheres = self.SplitSort(self.CentroidPoint)

        for i in range(2):
            for j in range(2):
                for k in range(2):
                    spheres = SubSpheres[i][j][k]
                    assert len(self.Subtrees[i][j][k]) == 0
                    self.Subtrees[i][j][k].append(Octree(spheres, len(spheres)))

    def SplitSort(self, split_point):
        """
        Split the SphereList into 8 sublists
        Param:
        ---------------------------------------------------------------------------
            split_point: (3,) numpy array, the center of all the bounding spheres
        Return:
        ---------------------------------------------------------------------------
            sub_tree: store the divided Spheres
        """
        if len(self.Spheres) == 0 or len(self.Spheres) == 1:
            raise ValueError("SphereList must have more than 1 element to get a splited space")
        
        sub_tree = [[[[] for i in range(2)] for j in range(2)] for k in range(2)]

        if len(self.Spheres) == 2:
                sub_tree[0][0][0].append(self.Spheres[0])
                sub_tree[1][1][1].append(self.Spheres[1])
                return sub_tree
        
        for S in self.Spheres:
            i = 1 if split_point[0] < S.center[0] else 0
            j = 1 if split_point[1] < S.center[1] else 0
            k = 1 if split_point[2] < S.center[2] else 0
            sub_tree[i][j][k].append(S)
        return sub_tree
    
    def Centroid(self):
        """
        Find the centroid of the SphereList
        Param:
        ---------------------------------------------------------------------------
            None
        Return:
        ---------------------------------------------------------------------------
            centroid: 3x1 numpy array, the centroid of the centers of the bounding spheres
        """
        if len(self.Spheres) == 0:
            return None

        centroid = np.zeros(3)
        for S in self.Spheres:
            centroid += S.center
        return centroid/len(self.Spheres)
    
    def FindMaxCoordinates(self):
        """
        Find the maximum coordinate of the SphereList
        Param:
        ---------------------------------------------------------------------------
            None
        Return:
        ---------------------------------------------------------------------------
            UB: Upper Bound of all the spheres UB = [x_max, y_max, z_max]
        """
        if len(self.Spheres) == 0:
            return None

        UBs = np.vstack([S.center + S.radius for S in self.Spheres])
        UB = np.max(UBs, axis=0)
        return UB
    
    def FindMinCoordinates(self):
        """
        Find the minimum coordinate of the SphereList
        Param:
        ---------------------------------------------------------------------------
            None
        Return:
        ---------------------------------------------------------------------------
            LB: Lower bound of all the spheres LB = [x_min, y_min, z_min]
        """
        if len(self.Spheres) == 0:
            return None

        LBs = np.vstack([S.center - S.radius for S in self.Spheres])
        LB = np.min(LBs, axis=0)
        return LB
    
    def FindMaxRadius(self):
        """
        Find the maximum radius of the SphereList
        Param:
        ---------------------------------------------------------------------------
            None
        Return:
        ---------------------------------------------------------------------------
            max_radius: float, the maximum radius of the bounding spheres
        """
        if len(self.Spheres) == 0:
            return None

        max_radius = 0
        for S in self.Spheres:
            if S.radius > max_radius:
                max_radius = S.radius
        return max_radius
    
    def FindClosestPoint(self, point, update_container):
        """
        Find the closest point in the Tree to the given point through the recursive method
        Param:
        ---------------------------------------------------------------------------
            point: (3,) numpy array, the point to find the closest point
            bound: float, the bound of the distance between the point and one bouding sphere
        Return:
        ---------------------------------------------------------------------------
            closest_point: (3,) numpy array, the closest point in the SphereList
        """
        self.update_container = update_container
        if len(self.Spheres) == 0:
            return
        dist = self.MaxRadius + self.update_container['bound']
        if (point - self.UB > dist).any() or (self.LB - point > dist).any(): return
        if self.HaveSubtrees:
            for i in range(2):
                for j in range(2):
                    for k in range(2):
                        self.Subtrees[i][j][k][0].FindClosestPoint(point,self.update_container)
        else:
            for S in self.Spheres:
                self.UpdateClosest(S, point, self.update_container)

    def UpdateClosest(self, S, point, update_container):
        """
        Update the closest point and the distance
        Param:
        ---------------------------------------------------------------------------
            S: a Sphere object
            point: (3,) numpy array, the point to find the closest point
        Return:
        ---------------------------------------------------------------------------
            None
        """
        dist = np.linalg.norm(point - S.center)
        if dist - S.radius > update_container['bound']: return
        closest_point = S.FindClosestPoint2Triangle(point)
        dist = np.linalg.norm(closest_point - point)
        if dist < update_container['bound']:
            update_container['bound'] = dist
            update_container['closest_point'] = closest_point

if __name__=="__main__":
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],[1,1,0],[-1,0,0]])
    triangle_idx = np.array([[0, 1, 2],[1,2,3],[0,2,4]])


    # O = Octree(S, len(S))

    UB = np.array([0.5, 0.5, 0.5])
    LB = np.array([-0.5, -0.5, -0.5])

    print(((P-UB) < 0.25).any())



