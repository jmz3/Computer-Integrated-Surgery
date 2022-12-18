import numpy as np
from .FindBoundingSphere import SingleSphere
from .Octree import Octree


class FindClosestPoint2Mesh:
    """
    Find the closest point on the triangle to the given point P.
    Param:
    ---------------------------------------------------------------------------
    a: 3x1 numpy array, the point to be projected
    vertex: Nx3 numpy array, the vertices of the triangle, each column is a vertex
    Nfaces: the number of the triangles in the mesh
    face_idx: the index of the vertices of the triangles in the mesh

    Return:
    ---------------------------------------------------------------------------
        P_closest: 3x1 numpy array, the closest point on the mesh to P
    """

    def __init__(self, vertex, Nfaces, face_idx):
        # Initialize the mesh info
        self.vertex = vertex
        self.Nf = Nfaces
        self.face_idx = face_idx
        self.Spheres = self.FindBoundingSphereForMesh()
        self.octree = self.OctreeGenerator()

        # Initialize the values used in the Octree Solver

    def BruteForceSolver(self, a):
        """
        Find the closest point on the mesh to the given point P.
        Param:
        ---------------------------------------------------------------------------
            point: 3x1 numpy array, the point we use to find the closest point on the mesh
            self: a Mesh object that contains the vertices and faces index of the mesh

        Return:
        ---------------------------------------------------------------------------
            P_closest: 3x1 numpy array, the closest point on the mesh to P
        """
        # Size check
        if a.size != 3:
            raise ValueError("Input must be a (3,) numpy array")
    
        a.reshape(3)
        min_dist = np.inf  # Initialize the minimum distance to infinity
        for i in range(self.Nf):
            # Get the vertices of the triangle from the face index
            # p q r should be 3x1 numpy array
            p = self.vertex[self.face_idx[i, 0], :].reshape(3)
            q = self.vertex[self.face_idx[i, 1], :].reshape(3)
            r = self.vertex[self.face_idx[i, 2], :].reshape(3)

            # Find the closest point on the triangle to the given point
            h = self.FindClosestPoint2Triangle(a, p, q, r)
            # Update the minimum distance
            if np.linalg.norm(h-a) < min_dist:
                min_dist = np.linalg.norm(h-a)
                P_closest = h

        return P_closest, min_dist

    def FindClosestPoint2Triangle(self, a, p, q, r):
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
        SphereList = []
        for triangle_idx in range(self.Nf):
            # Get the vertices of the triangle from the face index
            # the vertices are stacked as row vectors i.e. 
            # [px py pz 
            #  qx qy qz 
            #  rx ry rz]
            vertex = self.vertex[self.face_idx[triangle_idx, :], :]

            # Find the bounding sphere of the triangle
            sphere_ = SingleSphere(vertex,triangle_idx)
            SphereList.append(sphere_)
            # print(sphere.center)
            # print(sphere.radius)
            # print(vertex)
            # for S in SphereList:
            #     print(S.triangle_idx)
        return SphereList

    def OctreeGenerator(self):
        """
        Generate the octree of the mesh
        Param:
        ---------------------------------------------------------------------------
            self: a Mesh object that contains the vertices and faces index of the mesh
        Return:
        ---------------------------------------------------------------------------
            octree: an Octree object that contains the bounding sphere of the mesh
        """
        octree_ = Octree(self.Spheres, len(self.Spheres))
        return octree_
    
    def OctreeSolver(self, a):
        """
        Find the closest point on the mesh to the given point P using Octree
        Param:
        ---------------------------------------------------------------------------
            a: 3x1 numpy array, the point we use to find the closest point on the mesh
            self: a Mesh object that contains the vertices and faces index of the mesh

        Return:
        ---------------------------------------------------------------------------
            P_closest: 3x1 numpy array, the closest point on the mesh to P
        """
        # Size check
        if a.size != 3:
            raise ValueError("Input must be a (3,) numpy array")
        
        container = {}
        container['bound'] = 1e3
        self.octree.FindClosestPoint(a, container)
        print(self.octree.update_container['sphere_idx'])
        return self.octree.update_container['closest_point'],\
               self.octree.update_container['bound'],\
               self.octree.update_container['sphere_idx']
        
        
        
if __name__ == "__main__":
    P = np.array([1, 0, 0.25])
    Q = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],[1,1,0],[-1,0,0]])

    # print(np.inner(P.reshape(3),P.reshape(3)))
    c_ = FindClosestPoint2Mesh(Q, 3, np.array([[0, 1, 2],[1,2,3],[0,2,4]]))
    c_.FindBoundingSphereForMesh()
    print(c_.Centroid())
    p1 = np.array([2.0,-1.0,0.0])
    print(np.max([p1,P],axis=0))
    SubSpheres = [[[[] for i in range(2)] for j in range(2)] for k in range(2)]
    print(len(SubSpheres[:][1]))

    x = 1 < 3
    print(x)
    # print(c_.BoundingSphere(np.array([0, 1, 2])))
