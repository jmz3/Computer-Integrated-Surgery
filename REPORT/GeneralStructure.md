1. load data and find dk
2. Brute force search -> class FindClosestPoint2Mesh.BruteForce

Import all triangles, stack them into a list, linear search ( using find closest point on tiangle )

3. Octree search -> class FindClosestPoint2Mesh.Octree

Import all triangles -> compute bouding spheres for all triangles -> find Centroid Point -> Split the Spheres into 8 quadrants -> Generate Subtrees -> Generat Bounding Box -> Find Closest Subtree -> Generate Bounding Box -> ... Until Subtree.length < 2 

Substep1: find Centroid Point: n triangles -> n spheres = center & radius ; $\Sigma$ center / n 

Substep2: Split: sphere.center.x ? Centroid point.x

Sphere.center.y ? centroid Point.y

z ? z



