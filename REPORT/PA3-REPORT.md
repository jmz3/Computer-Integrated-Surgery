 <h1 align="center">PA3-REPORT</h1>



## **I. Mathematics & Algorithms Implementation**

This section introduces the mathematical principles and implemented algorithm for the 6 problems in Programming Assignment 3.

### 0. Scienario

In the assignment of programming, we are asked to implement and use a simplified version of iterative-closest point registration. Our purpose is to find the closest points in the surface of the model to each point in the the point cloud. We have tired Brutal Search, Bounding SphereSearch,  and Octree Search.

In the programming assignment, we use LED trackers to detect 2 rigid bodies and obtain the point cloud from intraoperative reality. The CT system could obtain the information of the 3D surface model. 

### 1. Find A Tip with respect to B Frame

#### 1) Mathematical Method

Here we use the following workflow to implement to matching part of iterative closest points.
(1) get the body definition files, recorded as $a$ and $b$ ;

(2) for each data frame $k$,  we get body definition files, recorded as  $\vec A_{i,k}$  and   $\vec B_{i,k}$ ;

(3) We have rigid bodies $A$ and $B$. We put the rigid body $A$ as a pointer and keep its tip contact with the points on the bone's surface. We put the tip of rigid body $B$ screwing into the bone. We calculate the point cloud  {$\vec d_k$} according to the formula as follows.
$$
\vec d_k = F_{B,k}^{-1} F_{A,k} \vec A_{tip}
$$
where $A_{k}$ = $F_{A,k}a$  ， $B_{k}$ = $F_{B,k}b$ 

#### 2) Code Implementation

The code implemented in **"/PA3/pa3_compute_dk_test.py"**. Here we load data and find $\vec d_k$ . In the terminal, run the following command:

```bash

```

### 2. Find the Closest Point in Triangle

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

### 3. Find the Closest Point on Mesh - Linear Search

#### 1) Mathematical Method

For each point in {$\vec d_k$},  finding the closest triangle mesh equals to find the nearest point $\vec C_{k,i}$. The closest triangle could be find in many methods.  The methods such as Brute- Force Search, Simple Search with Bounding Spheres，and Search Based on Octree have been used in our assignment.

**1.1) Linear Search by Brute Force**

Build linear list of triangles and search for closest triangle to each point  $\vec d_k$. For every point  $\vec d_k$, we could use Brute-Force Search to compute the distance between $\vec d_k$ and its corresponding  $\vec C_{k,i}$, and we could find $\vec C_{k,i}^{nearest}$ .

| Algorithm | Linear Search by Brute Force                                 |
| --------- | ------------------------------------------------------------ |
| Step. 1   | minimum distance ←$∞$                                        |
| Step. 2   | **for** $i =$ 1 to number of triangles do                    |
| Step. 3   | $\vec C_{k,i}^{nearest}$ ← findTheClosestPoint($\vec d_k$, $triangle_i$) |
| Step. 4   | $\vec d_k=||\vec d_k-\vec C_{k,i}^{nearest}||$               |
| Step. 5   | **if** $\vec d_k\leq$minimum distance **then**               |
| Step. 6   | minimum distance←$\vec d_k$                                  |
| Step. 7   | closest point←$\vec C_{k,i}^{nearest}$                       |
| Step. 8   | **end if**                                                   |
| Step. 9   | **end for**                                                  |

**1.2) Linear Search by Bounding Spheres**

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

2. Simple Search with Bounding Spheres, the pseudo code of the algrithom is shown as follows.

| Algorithm | Linear Search by Bounding Spheres         |
| --------- | ----------------------------------------- |
| Step. 1   | bound ←$∞$                                |
| Step. 2   | **for** $i =$ 1 to number of triangles do |
| Step. 3   | **if** $||q_i-a||-r_i\leq bound$ **then** |
| Step. 4   | $c_i=||\vec d_k-\vec C_{k,i}^{nearest}||$ |
| Step. 5   | **if**  $d_i\leq bound$  **then**         |
| Step. 6   | bound←$d_i$                               |
| Step. 7   | closest point←$c_{i}$                     |
| Step. 8   | **end if**                                |
| Step. 9   | **end if**                                |
| Step. 10  | **end for**                               |

#### 2) Code Implementation

**2.1) Linear Search by Brute Force**

The code implemented in **"/PA3/pa3_.py"**.

```bash

```

##### **2.2) Linear Search by Bounding Spheres**

The code implemented in **"/PA3/pa3_.py"**.

```bash

```



### 4 . Find Closest Point on Mesh - Octree Search

#### 1) Mathematical Method

The algorithm for Octree Search has two main parts: the tree construction and the closest point detection.  We have implemented a Python according to the slides in the class.

| Algorithm | Octree Search                                                |
| --------- | ------------------------------------------------------------ |
| Step. 1   | Import all triangles                                         |
| Step. 2   | Compute bounding spheres for all triangles                   |
| Step. 3   | Find Centroid Point                                          |
|           | N triangles w.r.t N spheres with its own center and radius; Add all N centers together and then divided by N to find the Centroid Point of the sphere |
| Step. 4   | Split the Spheres into 8 quadrants                           |
|           | Compare Sphere.Center.x and Centroid Point.x:  if Sphere.Center.x<Centroid Point.x, divide the Sphere into 2 parts; |
|           | Compare Sphere.Center.y and Centroid Point.y: if Sphere.Center.y<Centroid Point.y, divide the Sphere into 2 parts; |
|           | Compare Sphere.Center.z and Centroid Point.z: if Sphere.Center.z<Centroid Point.z, divide the Sphere into 2 parts; |
| Step. 5   | Generate Subtrees                                            |
| Step. 6   | Find Closest Subtree                                         |
| Step. 7   | Generate Bounding Box                                        |
|           | ...                                                          |
|           | **if**  Subtree.length < 2                                   |
|           | **endif**                                                    |


#### 2) Code Implementation

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
    │   ├── **pa3_compute_dk_test.py**	  # compute the sample points input file
    │   ├── **pa3_step2_test.py**
    │   └── **pa3_linear_search_test.py**	 # Linear search test to find closest point on mesh
    └── **cispa** 			# Functions are contained in this directory
        ├── **CarteFrame.py** 				#
        ├── **ComputeExpectValue.py** 		#
        ├── **CorrectDistortion.py** 		#
        ├── **DataProcess.py** 				# Contains useful functions like skew operation
        ├── **FindBoundingSphere.py**
        ├── **FindClosestPoint2Mesh.py**	   #  Find the closest point on the triangle to the given point P.
        ├── **FindClosestPoint2Triangle.py**  # Find the closest point on the triangle to the given point P.
        ├── **HomoRepresentation.py**    	#
        ├── **PivotCalibration.py**		 # Contains function : calib_pivot_points(F)
        └── **Registration.py**				# Contains function : regist_matched_points(X,Y)

## III. Unit Test and Debug

**NOTE**: Please change your directory to the ${PROGRAMS} directory before you start the unit test. In our case, the command is:

```bash
$ cd ~/*PARENT DIR*/PROGRAMS
```

And list all files to check whether you are in the correct directory.

```bash
$ ls
PA3   cispa
```

### 1. Verification for Compute dk Test

This script is to use to find closest point on mesh routine that works by linear search through all the triangles. In the ../PROGRAMS directory, run test script "/PA3/pa3_step1_test.py". In the terminal, run the following command:

```bash

```

### 2. Verification for Linear Search Test

This script is to use to find closest point on mesh routine that works by linear search through all the triangles. In the ../PROGRAMS directory, run test script "/PA3/pa3_linear_search_test.py". In the terminal, run the following command:

```bash

```

#### 1) Verification for Compute Bounding Sphere(vertex)

As shown in Fig.6. , the input vertices form a triangle in the x-y plane. The bounding sphere from our unit test program is consistent with the geometric relationship in the Fig.6.

```bash

```

#### 2) Verification for Compute Closest Point(point, vertex)

To test the effectiveness of the function, we conduct for 4 situations,  where $\vec c$ in the inside of the triangle,  $\vec c$ in the outside of the triangle,  $\vec c$ in the vertices of the triangle and  $\vec c$  on the side of the triangle.  We could visualize the results  from the test function.

```bash

```

### 3. Discussion for the results of finding point pairs in different cases

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

|  Method  | Running time(s) |      |      |      |      |
| :------: | --------------- | ---- | ---- | ---- | ---- |
|          | A               | B    | C    | D    | E    |
|  Brutal  |                 |      |      |      |      |
| BoundSp* |                 |      |      |      |      |
|  Octree  |                 |      |      |      |      |

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

## IV. Result and Discussions





## Contributions

Jiaming Zhang developed method ; Chongjun Yang developed other methods of this PA. Both team member complete a part of this report.



## References

[1] 

[2] 













