 <h1 align="center">PA3-REPORT</h1>

![Screen Shot 2022-10-25 at 9.07.17 PM](/Users/jeremy/Library/CloudStorage/OneDrive-Personal/601.655CIS1/Homework/ProgramAssignment/REPORT/PA3-REPORT.assets/Screen Shot 2022-10-25 at 9.07.17 PM.png)

## **I. Mathematics & Algorithms Implementation**

This section introduces the mathematical principles and implemented algorithm for locating the closest point to a given mesh file described in Programming Assignment 3.

### 0. Scienario

In the assignment of programming, a simplified version of Iterative Closest Point (ICP) has been developped. Our purpose is to find the closest points in the surface of the model to each point in the the point cloud. A naive search approach has been implemented, and the performance is tested based on the running time consumption. To speed up the searching process, we introduced  Octree structure. Both methods have been tested and validated based on given data.

In the programming assignment, we use LED trackers to detect 2 rigid bodies and obtain the point cloud from intraoperative reality. The CT system could obtain the information of the 3D surface model. <img src="/Users/jeremy/Library/CloudStorage/OneDrive-Personal/601.655CIS1/Homework/ProgramAssignment/REPORT/PA3-REPORT.assets/Screenshot 2022-11-20 at 3.03.26 AM.png" alt="Screenshot 2022-11-20 at 3.03.26 AM" style="zoom:50%;" />

The mathematical steps followed are described in this section.

### 1. Find A Tip with respect to B Frame

#### 1) Mathematical Method

Given two fiducials with known rigid markers sticked on them, we want to find one fiducial described in the local frame of another fiducial rigid body. First, get the body definition files, recorded as $a$ and $b$ ; Second, for each data frame $k$,  we get body definition files, recorded as  $\vec A_{i,k}$  and   $\vec B_{i,k}$ ; Then, we read rigid body description file and import them as $A$ and $B$. We put the rigid body $A$ as a pointer and keep its tip contact with the points on the bone's surface. We put the tip of rigid body $B$ screwing into the bone. We calculate the point cloud  {$\vec d_k$} according to the formula as follows.

$$
\vec d_k = F_{B,k}^{-1} F_{A,k} \vec A_{tip}
$$
where $A_{k}$ = $F_{A,k}a$  ， $B_{k}$ = $F_{B,k}b$ are derived from the point cloud to point cloud registration method introduce in prevoius programming assignments Ref [1](1).

#### 2) Code Implementation

The code implemented in **"/PA3/pa3_compute_dk_test.py"**. The basic steps are:

| Algorithm 1 | Compute dk                                                   |
| ----------- | ------------------------------------------------------------ |
| **Input**:  | Point Clouds: LED readings $A_k$ and $B_k$ , rigid body description file $A$ and $B$ |
| **Output**: | $A_{tip}$ position w.r.t. $B$ body frame, i.e. $d_k$ |
|  | Load $A_k$ , $B_k$ , $A$ and $B$ |
|  | **FOR** each frame in { $A_k , B_k$} |
|  | **CALL** $F_{A,k}$ = Registration($A_k$ , $A$) $F_{B,k}$ = Registration($B_k$ , $B$) |
|  | **COMPUTE** $\vec d_k = F_{B,k}^{-1} F_{A,k} \vec A_{tip}$ |
|  | **ENDFOR** |



### 2. Find the Closest Point in Triangle

#### 1) Mathematical Method

According to the notes in class,  the closest point $\vec C_{k,i}$would be calculated as follows 

<img src="/Users/jeremy/Library/CloudStorage/OneDrive-Personal/601.655CIS1/Homework/ProgramAssignment/REPORT/PA3-REPORT.assets/Screenshot 2022-11-20 at 4.05.40 AM.png" alt="Screenshot 2022-11-20 at 4.05.40 AM" style="zoom:50%;" />

For the system as shown in the above figure, $\vec {ac}$ can be derived from 
$$
\vec{ac} = (\vec a-\vec p) -( \lambda (\vec q-\vec p)+\mu(\vec r-\vec p))
$$

Since the closest point c must ensure $\vec{ac}$ to be the shortest vector among all possible choices of point c. The above functions could be rewritten as a Least Squares form as follows:
$$
\begin{bmatrix} \vec q-\vec p & \vec r-\vec p \end{bmatrix}  \begin{bmatrix} \lambda\\\mu\end{bmatrix}
\approx\vec a-\vec p
$$
Hence, by solving the Least Squares question, $\lambda$ and $\mu$ could be obtained. According to the  geometric relationship in Fig.1, the function could be written as follows.

$$
\vec c=\vec p+λ(\vec q-\vec p)+µ(\vec r-\vec p)
$$

If $\lambda\geq0,\mu\geq0,\lambda+\mu\leq1$, then $\vec c$ locates within the triangle and become the closest point. ![Screenshot 2022-11-20 at 4.13.01 AM](/Users/jeremy/Library/CloudStorage/OneDrive-Personal/601.655CIS1/Homework/ProgramAssignment/REPORT/PA3-REPORT.assets/Screenshot 2022-11-20 at 4.13.01 AM.png)Otherwise,  we have to find a point on the edge of the triangle. There are 3 different regions divided by the $\lambda$ and $\mu$. For every case, we can project the computed c on a certain edge to find the actual closest point. The projection $c^*$is shown in the following figure: 

<img src="/Users/jeremy/Library/CloudStorage/OneDrive-Personal/601.655CIS1/Homework/ProgramAssignment/REPORT/PA3-REPORT.assets/IMG_0597.jpg" alt="IMG_0597" style="zoom:50%;" />

where $\alpha$ is given by:
$$
\alpha = Max(0, Min(\frac{(c-p) \cdot (q-p)}{(q-p)\cdot (q-p)},1))
$$
Therefore, $c^*$ can be established by the linear combination of p and q:
$$
c^* = p + \alpha \times (q-p)
$$
Now, replace $c$ with $c^*$, we will be able to find the closest point to a specific triangle all the time.

#### 2) Code Implementation

The code implemented in **"/cispa/FindClosestPoint2Triangle.py"** and the test script is developped in **"/PA3/pa3_find_closest_on_triangle.py"**. The steps are as following:

| Algorithm 2 | Find Closest Point to Triangle                               |
| ----------- | ------------------------------------------------------------ |
| **Input**:  | Point $a$ and Triangle $T = \{p,q,r\}$                       |
| **Output**: | Closest Point $h$                                            |
|             | **CALL** $ \begin{bmatrix} \lambda & \mu\end{bmatrix}^T$ = LeastSquare($\begin{bmatrix} \vec q-\vec p & \vec r-\vec p \end{bmatrix} ,  \vec a-\vec p$) |
|             | **COMPUTE**: $h$ = $ p + \lambda (q-p) + \mu(r-p)$           |
|             | **IF** $\lambda < 0$:                                        |
|             | &nbsp; $h$ = ProjectOnSegment($h,r,p$)                       |
|             | **ELIF** $\mu < 0$:                                          |
|             | &nbsp; $h$ = ProjectOnSegment($h,p,q$)                       |
|             | **ELIF** $\lambda + \mu > 1$:                                |
|             | &nbsp; $h$ = ProjectOnSegment($h,q,r$)                       |
|             | **ENDIF**                                                    |
| **RETURN**: | $h$ is the closest point on the triangle                     |



### 3. Find the Closest Point on Mesh - Linear Search

#### 1) Mathematical Method

For each point in {$\vec d_k$},  finding the closest triangle mesh equals to find the nearest point $\vec C_{k,i}$. The closest triangle could be find in many methods.  The methods such as Brute-Force Search, and Octree-based Search have been used in our assignment.

Build linear list of triangles and search for closest triangle to a given point. By calling FindClosestPoint2Triangle function introduced in subsection 2, we can get a set of closest points for all triangles in the given mesh. Compare the minimum distance for all triangles and find the closest one. This method is called Brute-Force Search.

#### 2) Code Implementation

The code implemented in**"cispa/FindClosestPoint2Mesh.py"** and the test script is  **"/PA3/pa3_linear_search_test.py"**.

| Algorithm3 | Linear Search by Brute Force                                 |
| ---------- | ------------------------------------------------------------ |
| **INPUT**  | Vertices Coordinates and Triangle Indices                    |
| **OUTPUT** | A tuple that contains closest point and its corresponding distance |
|            | minimum distance = $\infty$                                  |
|            | **FOR** $i =$ index of the triangles                         |
|            | &nbsp; **CALL**: $\vec C_{k,i}^{nearest}$ = FindClosestPoint2Triangle($\vec d_k$, $triangle_i$) |
|            | &nbsp; $\vec d_k=||\vec d_k-\vec C_{k,i}^{nearest}||$        |
|            | &nbsp; **IF** $\vec d_k\leq$minimum distance **then**        |
|            | &nbsp; &nbsp; minimum distance =$\vec d_k$                   |
|            | &nbsp; &nbsp; closest point = $\vec C_{k,i}^{nearest}$       |
|            | &nbsp; **ENDIF**                                             |
|            | **ENDFOR**                                                   |
| **RETURN** | [minimum distance, closest point]                            |



### 4. Generate Bounding Spheres

Build bounding spheres around each triangle and with the help of these spheres, we could reduce the number of careful checks required. The working flow is as follows. The scenario to find bounding sphere for each mesh triangle is shown as follows

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
with $(\lambda\vec d-\vec v)^2\leq(\lambda\vec d-\vec u)^2$. We could simplified as follows:
$$
λ\geq\frac{\vec v^2-\vec u^2}{2d(\vec v-\vec u)}=γ
$$
If$γ\leq0$ , then just pick $\lambda\leq0$. Otherwise, pick $\lambda=γ$.

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
    │   ├── **pa3_find_closest_on_triangle.py** 
    │   ├── **pa3_octree_search_test.py**
    │   └── **pa3_linear_search_test.py**	 # Linear search test to find closest point on mesh
    └── **cispa** 			# Utility Functions are contained in this directory
        ├── **CarteFrame.py**
        ├── **ComputeExpectValue.py** 
        ├── **CorrectDistortion.py** 	
        ├── **DataProcess.py** 				# Import and Export Data
        ├── **FindBoundingSphere.py**
        ├── **FindClosestPoint2Mesh.py** 	# Find the closest point on the mesh to point P.
        ├── **FindClosestPoint2Triangle.py**	# Find the closest point on the triangle to P.
        ├── **HomoRepresentation.py**
        ├── **Octree.py** 							# Generate the octree based on given mesh
        ├── **PivotCalibration.py**
        └── **Registration.py**				

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

**Dear Graders**, Before starting to run our code, please remember keep matplotlib in your virtual environment. 

### 1. Verification for Compute dk Test

This script is to use to find closest point on mesh routine that works by linear search through all the triangles. In the ../PROGRAMS directory, run test script "/PA3/pa3_compute_dk_test.py". In the terminal, run the following command:

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

|       Method        | Time(s) | | | | | |
| :-----------------: | --------------- | -------- | -------- | -------- | --------- | -------- |
|                     | A               | B        | C        | D        | E         | F        |
| Brutal Force Solver | 1.4505          | 1.4548   | 1.4521   | 1.4238   | 1.4501    | 1.4422   |
|    Octree Solver    | 0.135787        | 0.162758 | 0.161032 | 0.129104 | 0.1443400 | 0.099773 |

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

### 1.A-Debug

The result is compared  with the "PA3-A-Debug-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 1 PA3-A-Debug-own-output</h6>

|      | OUR RESULTS                                                  | GIVEN RESULTS                                                |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|      | (17.90666   25.32838  -18.84287   17.90731   25.32410  -18.84304    0.00434)<br/>( -39.30009  -22.32570  -30.75257  -39.30331  -22.32773  -30.75337    0.00389)<br/>  (-9.82757  -13.44235   -1.22125   -9.82755  -13.44231   -1.22128    0.00005)<br/> | (17.91    25.32   -18.84        17.91    25.32   -18.84     0.000)<br/>  (-39.30   -22.33   -30.75       -39.30   -22.33   -30.75     0.000)<br/>   (-9.83   -13.44    -1.22        -9.83   -13.44    -1.22     0.000)<br/> |

### 2.B-Debug

The result is compared  with the "PA3-B-Debug-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 2 PA3-B-Debug-own-output</h6>

|      | OUR RESULTS                                                  | GIVEN RESULTS                                                |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|      | (1.15508   11.01661   -8.29899    1.18421   10.94645   -8.26507    0.08320)<br/>(-31.61757   -5.26328  -12.85190  -32.46366   -3.96838   -9.86385    3.36468)<br/>(-24.43960   -0.75184  -14.42152  -25.33479    0.98430  -12.44331    2.78008)<br/> | (1.15    11.01    -8.30         1.18    10.95    -8.27     0.082)<br/>  (-31.62    -5.26   -12.85       -32.46    -3.97    -9.86     3.366)<br/>  (-24.44    -0.76   -14.42       -25.34     0.98   -12.44     2.784)<br/> |

### 3.C-Debug

The result is compared  with the "PA3-C-Debug-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 3 PA3-C-Debug-own-output</h6>

|      | OUR RESULTS                                                  | GIVEN RESULTS                                                |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|      | (27.51716   -8.17727    6.19635   27.51501   -8.14967    6.18664    0.02933)<br/>(-8.77465    6.25957   46.45507   -6.95442    6.25081   46.40286    1.82100)<br/>(27.63058  -11.79476  -12.06240   27.82258  -12.01527  -11.99464    0.30014)<br/> | (27.52    -8.18     6.20        27.51    -8.15     6.19     0.030)<br/>   (-8.78     6.25    46.45        -6.95     6.25    46.40     1.823)<br/>   (27.63   -11.80   -12.06        27.82   -12.02   -12.00     0.298)<br/> |

### 4.D-Debug

The result is compared  with the "PA3-D-Debug-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 4 PA3-D-Debug-own-output</h6>

| OUR RESULTS                                                  | GIVEN RESULTS                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| (15.42274   19.93007   35.51362   15.44614   21.76373   36.08316    1.92021)<br/>   (-5.15204  -18.60613   -8.49794   -5.19629  -18.17795   -8.68783    0.47048)<br/>    (3.33449   22.29966   27.79139    3.45898   23.17946   27.91469    0.89708)<br/> | ( 15.42    19.93    35.51        15.45    21.76    36.08     1.922)   (-5.15   -18.60    -8.50        -5.19   -18.18    -8.69     0.470)<br/>    (3.34    22.29    27.79         3.46    23.18    27.92     0.902)<br/> |

### 5.E-Debug

The result is compared  with the "PA3-E-Debug-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 5 PA3-E-Debug-own-output</h6>

|      | OUR RESULTS                                                  | GIVEN RESULTS                                                |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|      | ( -2.28196   -5.06556  -27.61830   -0.78391   -4.32855  -28.66912    1.97271) <br/> (-41.51150  -13.61948  -42.49447  -38.68369  -13.27770  -40.83609    3.29599) <br/>  (30.09727   15.19825   -6.44416   33.02990   17.64569   -6.29841    3.82251)<br/> | (-2.28    -5.07   -27.62        -0.78    -4.34   -28.67     1.976)<br/>  (-41.51   -13.61   -42.49       -38.68   -13.28   -40.84     3.296)<br/>   (30.09    15.21    -6.45        33.02    17.65    -6.30     3.816)<br/> |

### 6.F-Debug

The result is compared  with the "PA3-F-Debug-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 6 PA3-F-Debug-own-output</h6>

|      | OUR RESULTS                                                  | GIVEN RESULTS                                                |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|      | ( -9.27443  -29.37243  -38.63571  -10.81595  -27.29781  -37.41410    2.85878)<br/> (-38.37045    0.91475  -25.07241  -39.95337    1.99688  -25.06008    1.91750)<br/>(-5.78249  -29.91310  -19.62803   -7.35776  -27.20945  -20.30206    3.20085)<br/> | (-9.27   -29.37   -38.63       -10.81   -27.29   -37.41     2.856)<br/>  (-38.36     0.91   -25.07       -39.95     2.00   -25.06     1.924)<br/>   (-5.79   -29.91   -19.63        -7.36   -27.21   -20.30     3.195)<br/> |

### 7.G-Debug

The result is compared  with the "PA3-G-Unknown-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 7 PA3-G-Unknown-own-output.txt</h6>

|      | OUR RESULTS                                                  |
| ---- | ------------------------------------------------------------ |
|      | (-13.66093   12.38037   30.02801  -13.96138   11.96082   30.24026    0.55799)<br/>   (16.00902   24.41337    8.53591   16.55037   26.03651    9.01081    1.77572)<br/>    (9.75001   15.57415  -10.16029    9.92203   15.53096   -9.94482    0.27908)<br/> |

### 8.H-Debug

The result is compared  with the "PA3-H-Unknown-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 8 PA3-H-Unknown-own-output.txt</h6>

|      | OUR RESULTS                                                  |
| ---- | ------------------------------------------------------------ |
|      | ( 2.51675  -13.64304    8.64420    2.83203  -11.96537    8.47300    1.71560)<br/>   (-4.31288  -12.33908  -37.45976   -2.14691  -12.27012  -38.16811    2.27991)<br/>  (-35.97575   -7.62909  -42.13914  -36.66262   -7.35385  -42.91870    1.07483)<br/> |

### 9.J-Debug

The result is compared  with the "PA3-J-Unknown-own-output.txt" and shown in the table below.

<h6 align="center">TABLE 9 PA3-J-Unknown-own-output.txt</h6>

|      | OUR RESULTS                                                  |
| ---- | ------------------------------------------------------------ |
|      | (21.48054   -0.92945   49.72730   20.59832   -0.49203   49.58690    0.99467)<br/>   (25.36021    6.03934   25.74393   27.53223    5.82745   26.43426    2.28892)<br/>    (8.03087   10.08615  -11.70728    7.97719   10.55353  -11.99916    0.55365)<br/> |

## Contributions

Jiaming Zhang developed method ; Chongjun Yang developed other methods of this PA. Both team member complete a part of this report.



## References

[1] Jiaming Zhang, Chongjun Yang. PA1-REPORT

[2] Jiaming Zhang, Chongjun Yang. PA2-REPORT

[3] https://en.wikipedia.org/wiki/Octree









