 <h1 align="center">PA3-REPORT</h1>

## **I. Mathematics & Algorithms Implementation**

This section introduces the mathematical principles and implemented algorithm for the 6 problems in Programming Assignment 3.

### 0. Scenario

#### 1) Mathematical Method

In the assignment of programming, we are asked to implement and use a simplified version of iterative-closest point registration. Our purpose is to find the closest points in the surface of the model to each point in the the point cloud. We have tired <u>Brutal Search, Boundning SphereSearch, KDtree Search and Octree Search</u> and compare the efficiency of the four methods.

In the programming assignment, we use EM trackers to detect 2 rigid bodies and obtain the point cloud from intraoperative reality. The CT system could obtain the information of the 3D surface model. We propose the steps of the workflow as follows:

1. Obtain the cloud in the intraoperative reality

   We have rigid bodies A and B. We put the rigid body A as a pointer and keep its tip contact with the points on the bone's surface. . We put the tip of rigid body B screwing into the bone. We calculate the point cloud  {$\vec d_k$} according to the formula as follows.
   $$
   \vec d_k = F_{B,k}^{-1} F_{A,k} \vec A_{tip}
   $$

2. Find the closest point $\vec c_k$ in a triangle mesh

   In the triangle, we intend to find a point whose distance to a point in the point cloud is the closest. We define the distance as the distance between a point in the point cloud and a triangle mesh. For every $\vec d_k$, we have a point set {$\vec C_{k,i}$}, where *i* is the triangle mesh index.

3. Find the closest point pairs{ $\vec d_k$, $\vec C_{k,i}^{nearest}$  }

   For every point in {$\vec d_k$}, finding the closest triangle mesh equals to find the nearest point $\vec C_{k,i}$. We search the {$\vec C_{k,i}$} space  to find the closest triangle mesh  $\vec C_{k,i}^{nearest}$ related to $\vec d_k$. After traversing the whole point cloud, the point pairs {$\vec d_k$, $\vec C_{k,i}^{nearest}$} would be obtained.

#### 2) Code Implementation

The code implemented in **"/PA3/pa3_.py"**.

```bash

```



### 1. Find the closest point in triangle

#### 1) Mathematical Method

According to the notes in class,  the closest point $\vec C_{k,i}$would be calculated as follows 

1. For the system as shown in Fig. 1. the function keeps right as follows

$$
\vec a-\vec p\approxλ(\vec q-\vec p)+µ(\vec r-\vec p)
$$

The above functions could be rewritten as a Least Squares form as follows:
$$
\begin{bmatrix} \vec q-\vec p & \vec r-\vec p \end{bmatrix}  \begin{bmatrix} λ\\µ\end{bmatrix}
\approx\vec a-\vec p
$$
Hence, by solving the Least Squares question, λ and µ could be obtained.

2. According to the  geometric relationship in Fig.1, the function could be written as follows.

$$
\vec c=\vec p+λ(\vec q-\vec p)+µ(\vec r-\vec p)
$$

If $λ\geq0,µ\geq0,λ+µ\leq1$, then $\vec c$ locates within the triangle and become the closest point. Otherwise,  we have to find a point on the border of the triangle. Here we define $\vec c^{*}$ as the project of $\vec c$  on the border of the triangle and as the closest point.  We could calculate  $\vec c^{*}$ according to Fig.2. and Fig.3.

3. For each $\vec d_k$,  the closest point of each triangle mesh could be found according to  the above steps, here we get the point set{$\vec C_{k,i}$}.

#### 2) Code Implementation

The code implemented in **"/PA3/pa3_.py"**.

```bash

```



### 2. Find the closest triangle

#### 1) Mathematical Method

For each point in {$\vec d_k$},  finding the closest triangle mesh equals to find the nearest point $\vec C_{k,i}$. The closest triangle could be find in many methods.  The methods such as <u>Brute- Force Search, Simple Search with Bounding Spheres, Search Based on KD tree, and Search Based on Octree</u> have been used in our assignment.

**1.1) Brute-Force Search**

Build linear list of triangles and search for closest triangle to each point  $\vec d_k$. For every point  $\vec d_k$, we could use Brute-Force Search to compute the distance between $\vec d_k$ and its corresponding  $\vec C_{k,i}$, and we could find $\vec C_{k,i}^{nearest}$ .

| Algorithm | linear search by brute force                                 |
| --------- | ------------------------------------------------------------ |
| INPUT     |                                                              |
| OUTPUT    | minimum distance ←$∞$                                        |
|           | **for** $i =$ 1 to number of triangles do                    |
|           | $\vec C_{k,i}^{nearest}$ ← findTheClosestPoint($\vec d_k$, $triangle_i$) |
|           | $\vec d_k=||\vec d_k-\vec C_{k,i}^{nearest}||$               |
|           | **if**                                                       |
|           |                                                              |
|           |                                                              |



**1.2) Simple Search with Bounding Spheres**

Build bounding spheres around each triangle and with the help of these spheres, we could reduce the number of careful checks required. The working flow is as follows.

1. The scenario to find bounding sphere for each mesh triangle is shown as Fig.4.

   

   Fig. 4.

Here we assume edge $\vec a-\vec b$ is the longest. Then the center $\vec q$ of the sphere will work as follows.
$$
(\vec b-\vec q)(\vec b-\vec q)=(\vec a-\vec q)(\vec a-\vec q)
$$

$$
(\vec c-\vec q)(\vec c-\vec q)\leq(\vec a-\vec q)(\vec a-\vec q)
$$

$$
(\vec b-\vec a)\times(\vec c-\vec a)\cdot(\vec q-\vec a)=0
$$

We could caculate the radius and center as follows.

(1) Compute
$$
\vec f=\frac{(\vec a+\vec b)}{2}
$$
(2) Define
$$
\vec u=\vec a-\vec f;\vec v=\vec c-\vec f;\vec d=(\vec u\times\vec v)\times\vec u
$$
(3) Then the sphere center $\vec q$ lies along the line
$$
\vec q=\vec f+λ\vec d
$$
with $(λ\vec d-\vec v)^2\leq(λ\vec d-\vec u)^2$. We could simplified as follows:
$$
λ\geq\frac{\vec v^2-\vec u^2}{2d(\vec v-\vec u)}=γ
$$
If$γ\leq0$ , then just pick $λ\leq0$. Otherwise, pick $λ=γ$.

2. Simple Search with Bounding Bpheres, the pseudo code of the algrithom is shown in Fg. 5. as follows.

2. | Algorithm  | Simple Search with Bounding Spheres                          |
   | :--------: | :----------------------------------------------------------- |
   | **INPUT**  | Triangle $i$ has corners [$\vec p,\vec q,\vec r$]            |
   | **OUTPUT** | Surrounding sphere  $i$ has radius $ρ_{i}$ center $\vec q_{i}$ bound=$∞$ |
   |            | **FOR** i=1 to N                                             |
   |            | **IF** $|| \vec q_{i}-\vec a||-ρ_{i}\leq bound$              |
   |            | $\vec h=$FindClosestPoint($\vec a, [\vec p_{i},\vec q_{i},\vec r_{i}]$); |
   |            | **ENDIF**                                                    |
   |            | **IF** $||\vec h-\vec a||< bound$ then $ \vec q_{i}-\vec a$  |
   |            | $\vec c=\vec h$; $bound=||\vec h-\vec a||$                   |
   |            | **ENDIF**                                                    |
   |            | **ENDFOR**                                                   |

**1.3) Search Based on KDtree**

The algorithm of Search Based on KDtree is an improvement based on Brute-Force Search. Similar to  Brute-Force Search, we could compute every $\vec C_{k,i}$ corresponding to $\vec d_k$ . Then we could build a KDtree on the {$\vec C_{k,i}$}, and search for the point $\vec C_{k,i}$  in KDtree  corresponding to $\vec d_k$.

**1.4) Search Based on Octree**

The algorithm for Octree Search has two main parts: the tree construction and
the closest point detection.  We have implemented a Python according to the slides in the class.

| Algorithm |               Class Definition of Octree                |
| :-------: | :-----------------------------------------------------: |
|    1:     |                    classdef $Octree$                    |
|    2:     |                       Properties                        |
|           | split point, upper, lower, centers, index, child, depth |
|    3:     |                         Methods                         |
|           |                        Octree();                        |
|           |                  enlarge bound(this);                   |

*BoundingBaseNode(Spheres, nSpheres)* recursively constructs the tree. The member method *FindClosestPoint(v)*  find the closest point on the surface recursively.



#### 2) Code Implementation

**1.1) Brute-Force Search**

The code implemented in **"/PA3/pa3_.py"**.

```bash

```

**1.2) Simple Search with Bounding Spheres**

The code implemented in **"/PA3/pa3_.py"**.

```

```

**1.3) Search Based on KDtree**

The code implemented in **"/PA3/pa3_.py"**.

```

```

**1.4) Search Based on Octree**

The code implemented in **"/PA3/pa3_.py"**.

```

```



## **II. Overall Structure**

The overall structure for the ../PROGRAMS folder is described as follows:

└── **PROGRAMS**
    ├── **PA3**			  # Test scripts are contained in this directory
    │   ├── **Data**	   # This DIR contains all the provided data
    │   ├── **output**	# This DIR contains all the result of our program
    │   ├── **pa3_main.py**				# This is the main process that output the result 
    │   ├── **pa3_problem5_test.py**	# Unit test for EM probe calibration
    │   └── **pa3_problem6_test.py**	# Unit test for Optical probe calibration
    └── **cispa** 			# Functions are contained in this directory
        ├── **DataProcess.py** 				# Contains useful functions like skew operation
        ├── **HomoRepresentation.py** 
        ├── **LoadData.py**					# Load data from text file
        ├── **PivotCalibration.py**		 # Contains function : calib_pivot_points(F)
        └── **Registration.py**				# Contains function : regist_matched_points(X,Y)

## III. Unit Testing and Debugging

**NOTE**: Please change your directory to the ${PROGRAMS} directory before you start the unit test. In our case, the command is:

```bash
$ cd ~/*PARENT DIR*/PROGRAMS
```

And list all files to check whether you are in the correct directory.

```bash
$ ls
PA3   cispa
```

### 1. Verification for two important subroutines



#### 1) Verification for Compute Bouding Sphere(vertex)

As shown in Fig.6. , the input vertices form a triangle in the x-y plane. The bounding sphere from our unit test program is consistent with the geometric relationship in the Fig.6.

```bash

```



#### 2) Verification for Compute Closest Point(point, vertex)

To test the effectiveness of the function, we conduct for 4 situations,  where $\vec c$ in the inside of the triangle,  $\vec c$ in the outside of the triangle,  $\vec c$ in the vertices of the triangle and  $\vec c$  on the side of the triangle.  We could visualize the results  from the test function.

```bash

```



### 2. Discussion for the results of finding point pairs in different cases

#### 1) Debug case ####

Table 1 shows us  a quantitative evaluation of various proposed methods to find the
closest points. For both the Debug and Unknown cases, we compared the running time of the four methods to demonstrate the efficiency improvement.

In Table 2 , 'BoundSp' denoting the Bounding Sphere Search demonstrates the highest  speed in computing the paired points. Except for the KDtree, all the advanced methods improved the computation efficiency comparing with the naive Brutal search. Because a KDtree pipeline has to find all the closest points on single triangle mesh beforehand, the method has to repeat this step and thus  requires about 0.7s in this dataset when dealing with each point of the point cloud and efficiency has been degraded largely.  Secondly, compared with the Brutal Search, though the Octree Search has largely reduced running time, Bounding Sphere Search still performs better than the Octree Search Method. This could be caused by the performance in the implementation of python. During the process of the tree construction, there would be a series of calling and assignment operations for List type limiting the efficiency of Octree.

**Table 1**: Quantitative evaluation of the proposed methods' accuracy.  All the  methods come to same accuracy. The error has been computed in  <u>L2 Norm</u>

| Case | $d$ error | $c$ error | $mag$ error |
| :--: | :-------: | :-------: | :---------: |
|  A   |           |           |             |
|  B   |           |           |             |
|  C   |           |           |             |
|  D   |           |           |             |
|  E   |           |           |             |
|  F   |           |           |             |

**Table 2**:  Comparison of various methods in running time for Debug case A-F

|  Method  | Running time(s) |      |      |      |      |      |
| :------: | --------------- | ---- | ---- | ---- | ---- | ---- |
|          | A               | B    | C    | D    | E    | F    |
|  Brutal  |                 |      |      |      |      |      |
| BoundSp* |                 |      |      |      |      |      |
|  Octree  |                 |      |      |      |      |      |
|  KDtree  |                 |      |      |      |      |      |





#### 2) Unknown case

For Unknown cases, we save the results in ./output.

In the section, we also give time comparison. For the 3 cases,  the Octree based Search
method performs better than the Bounding Sphere Search method.

**Table 3**:  Comparison of various methods in running time for Debug case G-K

|  Method  | Running time(s) |      |      |      |
| :------: | --------------- | ---- | ---- | ---- |
|          | G               | H    | I    | J    |
|  Brutal  |                 |      |      |      |
| BoundSp* |                 |      |      |      |
|  Octree  |                 |      |      |      |
|  KDtree  |                 |      |      |      |



## IV. Result and Discussions





## Contributions

Jiaming Zhang developed method ; Chongjun Yang developed other methods of this PA. Both team member complete a part of this report.



## References

[1] 

[2] 















