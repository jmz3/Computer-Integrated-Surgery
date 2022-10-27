<h1 align="center">PA2-REPORT</h1>

<img src="/Users/jeremy/Library/CloudStorage/OneDrive-Personal/601.655CIS1/Homework/ProgramAssignment/REPORT/PA2_REPORT.assets/Screen Shot 2022-10-25 at 9.07.17 PM.png" alt="Screen Shot 2022-10-25 at 9.07.17 PM" style="zoom:275%;" />

## I. Mathematics & Algorithms Implementation

This section introduces the mathematical principles and implemented algorithm for the 6 problems in Programming Assignment 2.

### Scenario

The goal is to correct the distortion of the electromagnetic tracker and apply the corrected data to locate the fiducial markers in the CT coordinate system.

<a id="Scene"></a>

<img src="/Users/jeremy/Library/CloudStorage/OneDrive-Personal/601.655CIS1/Homework/ProgramAssignment/REPORT/PA2_REPORT.assets/Screen Shot 2022-10-25 at 9.57.21 PM.png" alt="Screen Shot 2022-10-25 at 9.57.21 PM" style="zoom:275%;" />

### 0. Registration and Calibration Recap

In Programming Assignment 1, we developped a math package to describe frame transformation in Cartesian Space. We also implemented point cloud to point cloud registration and pivot calibration methods. The mathematical methods and code implementation are explained and described in Ref [1](). Here we'll go through a quick recap for what we included in PA1.

#### 1) Cartesian Math Package

Rigid body transformations can be represented by matrix multiplcations. Suppose there are two Cartesian coordinate frames, $A$ and $B$. Position vectors in frame $A$ and frame $B$ is denoted as $\vec p_a$ and $\vec p_b$, respectively. $F_{AB} = \{R_{AB}, t_{AB}\}$ represents the rigid body transformation from frame $B$ to frame $A$. The relationship between $\vec p_a$ and $\vec p_b$ is $\vec p_b = R_{AB} \vec p_b + t_{AB}$ . 

To do this in a more compact way, we can expand the transformation and position vector to their homogeneous form, i.e.
$$
F_{AB} = \begin{bmatrix} R_{AB} & t_{AB} \\ \vec 0 & 1\end{bmatrix} ,& p = \begin{bmatrix} \vec p \\ 1 \end{bmatrix}
$$
Hence, the rigid body can be simply represented as $F_1 \cdot F_2$ and $F \cdot p$. A function that computes the homogeneous form of the given transformation frame of position vector is provided in the **"../cispa/CarteFrame.py"** . 

#### 2) Registration

Our implementation uses a non-iterative least-squares approach to match two sets of 3D points. Suppose we have two point sets and denote them as $P = \{p_i, i \in 1,2,...,N\}$ and $P'= \{p_i', i \in 1,2,...,N\}$, and they are representing the same rigid body described at different poses. Hence, they should follows: $p_i' \approx Rp_i + t$, in which the error is stated as: $ei = p_i'-Rp_i-t$ . We applied Singular Value Decomposition to find the $R$ and $t$ that minimize the following cost function:

$$
{\Sigma} ^2  = \sum_{i=1}^N|| p'_i - Rp_i-t||^2
$$
To solve the problem, we took a two step method, i.e. Solve $R$ first and find the corresponding $t$. To do this, we have to centralize the point sets to annihilate the coupling between rotation and translation as $p_{ic} = p_i - \bar p$, $p_{ic}' = p_i' - \bar p'$. Therefore, the problem reduces to:
$$
\hat R = \underset{R}{\operatorname{argmax}} \sum ||p_{ic}' - Rp_{ic}||^2
$$
And $t$ can be found simply by: 
$$
\hat t = \bar p_i' - \hat R \bar p_i
$$
The code is implemented in the **"../cispa/Registration.py"** and the function is defined as **regist_matched_points(X,Y)**. Here X and Y represent two point sets. 

#### 3) Pivot Calibration

The model of pivot calibration is described as: $\vec p_{pivot} = F_i \vec p_{t} = R_i \vec p_t + \vec p_i$
It holds for all point pairs on the tool. $F_i$ is the rigid transformation from tool coordinate frame to the EM tracker coordinate frame. The calculation of pivot vector can be described as a least square problem like Eq (5).
$$
\begin{bmatrix}... & ...\\R_i & -I\\...&...\end{bmatrix}
\begin{bmatrix}p_t \\ p_{pivot} \end{bmatrix}
=
\begin{bmatrix}... \\ -p_i \\... \end{bmatrix}
$$
By solving this least square problem, we can get the pivot vector $p_{pivot}$ . The code is implemented in the **"../cispa/PivotCalibration.py"**. The function is defined as **calib_pivot_points(F)**. Here F is a list of homogeneous transformations. 



### 2. Compute $ C^{expected}$

We compute the expected value of calibration object poistion w.r.t. the EM tracker coordinate system at each frame k in this section.

#### 1) Mathematical Method

As described in the Ref [1](), $\vec C^{expected}$ is computed based on the Optical tracker data and arguments of the rigid body (i.e. calibration object). The Optical tracker readings is recorded at each frame k. Here the frame k denotes the position of the calibration object. Therefore, the steps taken can be stated as:

First, we compute the transformation for EM tracker base and calibration object w.r.t. the Optical tracker.
$$
\vec D_i[k] = F_D[k] \vec d_i\\
\vec A_i[k] = F_A[k] \vec a_i 
$$
And then, through registration, $F_D$ and $F_A$ can be found, thereby the $\vec C^{expected}$ can be computed by:
$$
\vec C_i^{expected}[k] = F_D^{-1}[k] F_A[k] \vec c_i
$$
Here, $\vec a_i$, $\vec b_i$ and $\vec c_i$ are the position of markers w.r.t. the calibration object reference frame. They are constant values since the calibration object is a rigid body.

#### 2) Code Implementation

The code is implemented in **"../cispa/ComputeExpectValue.py"** and the function is **C_expected()**. The steps of our algorithm is shown as follows:

|Algorithm 1|ComputeExpectValue.py |
| :--- | :--- |
|**INPUT:**| Point cloud {$\vec A_i$ $\vec D_i$} Optical marker position w.r.t. Optical tracker |
|| Point cloud {$\vec a_i$ $\vec d_i$ $\vec c_i$} Optical & EM marker position w.r.t. the calibration object |
|**OUTPUT:**| Expected position of EM marker w.r.t. the EM tracker |
|| **FOR:** each frame of data in {$\vec A_i$ $\vec D_i$} |
|| &nbsp;  **CALL:**  $F_D[k]$ = Registration$( D[k], d)$,  $F_A[k]$ = Registration$( A[k], a)$ |
|| &nbsp;  **FOR:** each point in set {$c$} |
|| &nbsp;  &nbsp;  **COMPUTE:**  $\vec c_i^{expected}[k]$ = $F_D^{-1}[k] F_A[k] \vec c_i$ |
|| &nbsp;  **ENDFOR** |
|| **ENDFOR** |
|**RETURN:**| $C^{expected} = \{\vec c_i^{expected}[k]\}$ |



### 3. Distortion Correction

We developped an approach for correcting distorted data. The distortion is invoked by the EM tracker and is manifested as a slight change in the relative position between markers on the calibration object.

#### 1) Mathematical Method

The distortion is corrected by applying Bernstein Polynomials to fit the given data. For a scalar $u$ Bernstein Polynomials are defined by the Bernstein basis:
$$
B_{N,k}(u) = \begin{pmatrix} N \\ k \end{pmatrix} (1-u)^{N-k}u^k
$$
Where N denotes the highest order of the Bernstein basis. For 3D case, the Berstein basis will become the production of 3 independent basis of scalar. 
$$
B_{N,ijk}(\vec p)= [\begin{pmatrix} N \\ i \end{pmatrix} (1-p_x)^{N-i}p_x^i]*[\begin{pmatrix} N \\ j \end{pmatrix} (1-p_y)^{N-j}p_y^j]*[\begin{pmatrix} N \\ k \end{pmatrix} (1-p_z)^{N-k}p_z^k]
$$
And thereby the Bernstein Polynomials can be established as a linear combination of the basis in Eq [9](). For any distorted point $\vec p = [p_x, p_y, p_z]$ in 3D space, if the order of Bernstein Polynomials N is known, we can have the polynomials $\vec F(\vec p)$ arranged into a vector form:
$$
\vec F(\vec p)
= \begin{bmatrix} 
F_{000}(\vec p) & F_{001}(\vec p) &...&  F_{ijk}(\vec p) & ... & F_{NNN}(\vec p)
\end{bmatrix}
$$
It's easy to show that $\vec F(\vec p)$ is a $1 \times (N+1)^3$ row vector, where $i,j,k \in \{0,1,2 ... N\}$ and together with the "ground truth" $\vec p_g$  they follows:
$$
\vec F(\vec p)
\begin{bmatrix} 
c_{000}^x & c_{000}^y & c_{000}^z \\
c_{001}^x & c_{001}^y & c_{001}^z \\
... & ... & ...\\
c_{ijk}^x & c_{ijk}^y & c_{ijk}^z \\
... & ... & ...\\
c_{NNN}^x & c_{NNN}^y & c_{NNN}^z \\
\end{bmatrix}_{(N+1)\times 3}
=
\begin{bmatrix}
p_g^x & p_g^y & p_g^z
\end{bmatrix}
$$
Obviously, the position vectors need to be scaled to a bounding box with the range of $[0,1]$ to maintain numerical stability of the Berntein basis. That is, $\forall \vec p$ in distorted dataset or the ground truth dataset, $\vec p_s = ScaleToBox(\vec p)$. Note that here we consider the data to be anisotropy, which means we independently scale the data on every dimension instead of simply normalize the data. As we can see, Eq 11 is a underdetermined equation. If we have multiple points to be corrected, Eq 11 can become a least square problem, i.e.
$$
\begin{bmatrix}
\vec F(\vec p_1) \\
\vec F(\vec p_2) \\
\vec F(\vec p_3) \\
...
\end{bmatrix}
\begin{bmatrix}
... & ... & ...\\ 
c_{ijk}^x & c_{ijk}^y & c_{ijk}^z \\
... & ... & ...
\end{bmatrix}
=
\begin{bmatrix}
p_{1g}^x & p_{1g}^y & p_{1g}^z \\
p_{2g}^x & p_{2g}^y & p_{2g}^z \\
p_{3g}^x & p_{3g}^y & p_{3g}^z \\
... & ... & ...
\end{bmatrix}
$$
The "distortion correction coefficient" can be easily found by solving the least square problem described in Eq [12](). And we can rectify any point set that follows the same distortion pattern by the distortion correction coefficient afterwards.

#### 2) Code Implementation 

The code contains four separate modules, namely **Scale2Box(p)**, **Bernstein(p, order)**, **fit(p,p_g)** and **predict(p, coeff)**. These functions are implemeted in **"../cispa/CorrectDistortion.py"**. The steps taken are described as the following pseudocode.

| Function1   | Bernstein(p, ORDER)                                          |
| ----------- | ------------------------------------------------------------ |
| **INPUT:**  | A Point Cloud $P$                                            |
| **OUTPUT:** | Corresponding bernstein polynomial matrix $F(P)$             |
| **DEFINE:** | bern_basis(i,u) = Combination$(ORDER, i)(1-u)^{N-i}u^i$      |
|             | **FOR** each point $\vec p \in P$                            |
|             | &nbsp;  **FOR** each order i,j,k in ORDER                    |
|             | &nbsp;  &nbsp;  **CALL**: $F_{ijk}(\vec p)$ = bern_basis$(i,p_x)$bern_basis$(j,p_y)$bern_basis$(k,p_z)$ |
|             | &nbsp;  &nbsp;  $F(\vec p)$ = [$F(\vec p)$ , $F_{ijk}(\vec p)$ ] |
|             | &nbsp;  **ENDFOR**                                           |
|             | &nbsp;  $F(P) = \begin{bmatrix} F(P) \\ F(\vec p) \end{bmatrix}$ |
|             | **ENDFOR**                                                   |

As we discussed in the mathematics part, the data need to be scaled to the range of [0,1].

| Function2   | Scale2Box                                                    |
| ----------- | ------------------------------------------------------------ |
| **INPUT:**  | Original Point Cloud $P$                                     |
| **OUTPUT:** | Scaled Point Cloud $P^{scaled}$                              |
| **STEP1:**  | Find the minimum and maximum value on every dimension        |
| **STEP2:**  | $x_i^{scaled} = \frac{x_i - x_{min}} {x_{max} - x_{min}}$ ,  $y_i^{scaled} = \frac{y_i - y_{min}} {y_{max} - y_{min}}$ , $z_i^{scaled} = \frac{z_i - z_{min}} {z_{max} - z_{min}}$ |
| **RETURN:** | $\vec p^{scaled} = \{..., [x_i^{scaled} y_i^{scaled} z_i^{scaled}], ...\}$ |

Now we need to prepare a distorted data and its corresponding standard data to "train the model" through fit function.

| Function3   | fit                                                          |
| ----------- | ------------------------------------------------------------ |
| **INPUT:**  | Distorted Point Cloud $P$ and Expected Point Cloud $P^{expected}$ |
| **OUTPUT:** | Correction coefficient matrix $C$                            |
| **STEP1**:  | Scale both point cloud to box $\vec p_s = Scale2Box(\vec p)$ and $\vec p_s^{expected} = Scale2Box(\vec p^{expected})$ |
| **STEP2:**  | Find Berstein Polynomial Matrix $F(\vec p)$ = Bernstein($p_s$, order=5) |
| **STEP3:**  | Solve $C = leastsquare(F(\vec p),\vec p_s^{{expected}})$     |
| **RERURN:** | $C$                                                          |

Note that here we always need to scale both the distorted data and the expected/corrected data. After we find the $C$ through fitting process, we can rectify or, in other words, predict the distortion for other point cloud.

| Function4   | Predict                                                      |
| ----------- | ------------------------------------------------------------ |
| **INPUT:**  | Point Cloud $\vec p$ that needs to dewlap, Correction coefficient matrix $C$ |
| **OUTPUT:** | Corrected Point Cloud $\vec p^{corrected}$                   |
| **STEP1:**  | Scale the input point cloud to box $\vec p_s = Scale2Box(\vec p)$ |
| **STEP2:**  | $\vec p_s^{corrected} = Bernstein(\vec p_s) \cdot C$         |
| **STEP3:**  | Scale back $\vec p_s^{corrected} $ according to the max and min value of input point cloud |
| **RETURN:** | $\vec p^{corrected}$                                         |



### 4. Pivot Calibration

#### 1) Mathematical Method

The EM pivot calibaration procesure is the same as the method in Programming Assignment 1. The main method is shown recap section. The only difference here is we need to correct the distortion of the EM marker point cloud before performing the pivot calibration. 

We choose the first frame as the start frame and then establish a local coordinate system attached on the tool. We define the corrected local and EM system coordinates at the k-th frame as $g^{corrected}[k]$ and $G^{corrected}[k]$, respectively. Since the local reference frame is rigidly connected to the calibration probe. $g^{corrected}[k]$ remains the same when k varies. Hence $g^{corrected}[0]$ will be taken as the local reference frame to perform pivot calibration. And thereby the transformation between different poses of the probe can be found by solving the registration problem:
$$
\vec G_j^{corrected}[k] = F_G[k] \cdot \vec g_j^{corrected}[0]
$$
Finally, using the pivot calibration package, we get the pivot vector relative to the EM tracker system.

#### 2) Code Implementation

The code is implemented in **"../PA2/pa2_problem3_test.py"**. 


| Algorithm 1     | Dewraped EM probe calibration                                |
| :-------------- | :----------------------------------------------------------- |
| **INPUT:**      | "../\*-empivot.txt" , Distortion Correction Coefficient Matrix C |
| **OUTPUT:**     | Calibrated post position $p_{dimple}$ relative to EM tracker frame |
| **INITIALIZE:** | Empty sequence $F$                                           |
| **STEP1:**      | Perform distortion correction on $G_j[k]$ and get $G_j^{corrected}[k]$ |
| **STEP2:**      | Compute the local coordinate $g_j^{corrected}[0]$            |
| **STEP3:**      | For $k \in (0,N_{frames})$ perform registration for $F_G[k] = registration(g_j^c[0],G_j^c[k])$ |
| **STEP4:**      | Push back $F_G[k]$ to the end of F                           |
| **STEP5:**      | Perform pivot calibration for $\{\vec p_{dimple}, t_G\} = calibration(F)$ |
| **RETURN:**     | $\vec p_{dimple}$                                            |



### 5. Find Fiducials w.r.t. EM Coordinate System

To find the fiducial positions described in the EM coordinate system, we need to derive the corresponding $\vec p_{dimple}$. The basic steps are: use the probe to touch the fiducial marker and collect a series of point cloud relative to the EM tracker, and then move the probe to another fiducial and repeat the data collection process.

#### 1) Mathematical Methods

The first step we need to take is to correct the data collected by the em tracker. Then we perform pivot calibration to find the tip position w.r.t. the probe local frame $\vec p_{tip}$. By registration between the point clouds collected at different fiducials, we can calculate $F_T$ , which is the rigid transformation between probe local frame and EM tracker frame. Finally, the pivot position of the probe w.r.t. the EM tracker can be calculated through:
$$
\vec p_{pivot} = F_T\vec p_{tip}
$$
And since the probe pricks at the fiducial marker, the positions of the fiducials are therefore equals to the pivot position of the probe. Hence, we have:
$$
\vec B_i = \vec p_i^{pivot}
$$

#### 2) Code Implementation

The code is implemented in **"../PA2/pa2_problem4_test.py"**. This program first perform the exact same process as problem3, i.e. perform distortion corrected pivot calibration to find the tip position in the probe local frame. And then perform point cloud registration between k-th frame of EM tracker readings and the "centralized" point cloud used in problem3. Eventually, the fiducial positions w.r.t. the EM tracker frame are found through Eq 15. The pseudocode can be found at Section 7 Step 1-3.



### 6. Find Transformation EM-CT ($F_{reg}$)

The transformation between two coordinate systems can be derived through locating the same point cloud in these two coordinate systems. 

#### 1) Mathematical Method

Through the prevoius section, we locate the fiducials in the EM coordinate system, denote as $\vec B_i$ and the fiducials in the CT coordinate system are recorded as $\vec b_i$. Therefore they can be connected through the following equation:
$$
\vec b_i = F_{reg}\vec B_i
$$

#### 2) Code Implementation

The code is implemented in **"../PA2/pa2_problem5_test.py"**. Note the EM-related data must be dewraped in the beginning. This program first perform the exact same process as problem4. Thus the fiducials are located in the EM tracker coordinate system, and the same fiducial positions can be read directly from the CT coordinate system. By performing point cloud to point cloud registration, the transformation $F_{reg}$ will be found. The pseudocode can be found at Section 7 Step 1-4.



### 7. Locate probe tip w.r.t. CT Coordinate System

The probe tip position is directly read from EM tracker data. With the known frame transformation from EM tracker frame to the CT frame $F_{reg}$, we can translate the probe tip position into the CT coordinate system.

#### 1) Mathematical Method

Since there are EM markers sticked on the probe, we can always get the position of the probe tip w.r.t. EM tracker by $F_G$ , which is derived from registration between the current point cloud and the centralized reference point cloud, and the tip position in probe local frame $\vec p_{tip}$, which is determined in the pivot calibration process.
$$
\vec B_i = \vec p_{pivot}^{EM} = F_G \vec p_{tip}
$$
Through the previous steps, the frame transformation matrix $F_{reg}$ and the tip position in probe local frame $\vec p_{tip}$ have been determined. Therefore, the tip position described in CT frame is stated as:
$$
\vec b_i = F_{reg}B_i = F_{reg}F_G\vec p_{tip}
$$

#### 2) Code Implementation

The code is implemented in **"../PA2/pa2_problem6_test.py"**. Note the EM-related data must be dewraped in the beginning.

| Algorithm 2 | Locate the probe tip in CT frame                             |
| :---------- | :----------------------------------------------------------- |
| **INPUT:**  | "/..-empivot.txt" "/..-em-NAV.txt" "/..-em-fiducials.txt" "/..-ct-fiducials.txt" |
| **OUTPUT:** | Probe tip location $\vec p_{CT}$w.r.t. CT coordinate system  |
| **STEP1:**  | Perform distortion correction on em related data             |
| **STEP2:**  | Perform pivot calibration to find the probe tip position relative to its local frame |
| **STEP3:**  | Perform point cloud registration and pivot calibration to find the fiducials w.r.t. the EM coordinate system |
| **STEP4:**  | Perform point cloud registration to find the frame transformation between EM frame and CT frame |
| **STEP5:**  | Compute the probe tip position w.r.t EM tracker frame and transfer it to CT frame |
| **RETURN:** | probe tip position w.r.t. CT coordinate system $\vec b_i$    |

## II. Overall Structure



## III. Unit Test and Debug

**NOTE**: Please change your directory to the ${PROGRAMS} directory before you start the unit test. In our case, the command is:

```bash
$ cd ~/*PARENT DIR*/PROGRAMS
```

And list all files to check whether you are in the correct directory.

```bash
$ ls
PA1   cispa
```



### 1. Expected Value Computation Unit Test

This code is mainly to test the utility functions we developed for distortion correction tasks. In the ../PROGRAMS directory, run test script "/PA2/pa2_problem1_test.py". This script is designed to test the expected value computation function. In the terminal, run the following command:

```bash
../PROGRAMS $ python PA2/pa2_problem1_test.py
[04:33:14] INFO     Average error between expect c and output c is 0.008871273266628715   
```

Here we automatically run the pa2-debug-a data to test whether our code generates the correct result. The average error is less than 1e-2.

### 2. Distortion Correction Unit Test

This code is mainly to test the utility functions we developed for distortion correction tasks. In the ../PROGRAMS directory, run test script "/PA2/pa2_problem2_test.py". This script is desinged to test the distortion correction functions. In the terminal, run the following command:

```bash
../PROGRAMS $ python PA2/pa2_problem2_test.py
```

This program generates a "clean" ground truth data and adds some distortion (i.e. nolinear noise) to the data. Then it uses the distortion correction functions to try to dewrap the distortion. The result of the code is shown in the following

```bash
for a nonlinear distortion with 
mean[1.53534271 3.07068542 1.12839849] and std[1.63235053 3.26470107 1.29301809]
distortion correction error: 3.769195147072704
```



### 3. Pivot Calibration Debug

This script is to complete the pivot calibration task with distorted data. In the ../PROGRAMS directory, run test script "/PA2/pa2_problem3_test.py". In the terminal, run the following command:

```bash
../PROGRAMS $ python PA2/pa2_problem3_test.py
[04:36:56] INFO     Pt =                                                                          
                    [[ 94.46736444] [  0.82093938] [-32.78984302]]                                                                                                                   
                    Ppivot =                                                                       											[[201.4096262] [202.36598282] [203.71908308]] 
                    
                    Ppivot Error =
                    0.0002581
```

Here we automatically run the pa2-debug-a data to test whether our code generates the correct result. Compare to the given result, the error is acceptable.



### 4. Find Fiducials in EM Debug

This script is to compute the fiducial position in EM frame with distorted data. In the ../PROGRAMS directory, run test script "/PA2/pa2_problem4_test.py". In the terminal, run the following command:

```bash
../PROGRAMS $ python PA2/pa2_problem4_test.py
[04:44:00] INFO     p_pivot:
                    [[201.40962622]                  
                     [202.36598282]                           
                     [203.71908308]]                 
           INFO     Fiducial points w.r.t. the EM tracker base coordinate system:
           INFO     Fiducial 1:                                                                                       [[421.3415502 ]                  
                     [411.46408246]                 
                     [429.36276817]]                     
           INFO     Fiducial 2:
                    [[440.07647847]                  
                     [352.49965478]                 
                     [449.25518391]]                     
           INFO     Fiducial 3:
                    [[385.86605572]                  
                     [350.78688559]                 
                     [267.1768318 ]]                     
           INFO     Fiducial 4:
                    [[405.0514645 ]                  
                     [441.45201353]                                
                     [429.95288982]]                             
           INFO     Fiducial 5:                                                                                       [[380.03203121]                  
                     [442.48777602]                 
                     [347.01043969]]                 
           INFO     Fiducial 6:                                                                   
                    [[488.95843587]                  
                     [339.31270346]                               
                     [435.51277305]]
```

Here we automatically run the pa2-debug-a data to test whether our code generates the correct result.

### 5. Compute $F_{reg}$ Debug

This script is to compute the fiducial position in EM frame with distorted data. In the ../PROGRAMS directory, run test script "/PA2/pa2_problem5_test.py". In the terminal, run the following command:

```bash
../PROGRAMS $ python PA2/pa2_problem5_test.py
[04:49:32] INFO     The transformation between EM and CT is                                       
                     [[ 9.99999999e-01  3.55984474e-05  1.69539742e-05 -3.05019871e+02]
                     [-3.57661079e-05  9.99950058e-01  9.99401798e-03 -3.02519129e+02]
                     [-1.65973559e-05 -9.99401857e-03  9.99950058e-01 -2.51984203e+02]
                     [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]  
```

Here we automatically run the pa2-debug-a data. But the result cannot be evaluated until we finish the next step.

### 6. Compute tip position in CT Debug

This script contains the aforementioned 5 functionality that I developped for PA2. In the ../PROGRAMS directory, run test script "/PA2/pa2_problem6_test.py". In the terminal, run the following command:

```bash
../PROGRAMS $ python PA2/pa2_problem6_test.py
[04:57:40] INFO     p_ct =                                                                        
                    [[160.11204592  44.3991593   62.22582002]
                     [ 42.16228214 171.29050862  27.04694339]
                     [161.55541322  33.77509118  44.40739624]
                     [ 87.51449534  97.13951216 138.07214223]]        
           INFO     debug data =
                    [[160.11  44.4   62.23]                     
                     [ 42.16 171.29  27.05]          
                     [161.56  33.77  44.41]          
                     [ 87.51  97.14 138.07]]         
           INFO     Total error = 0.010767916237550293 
```

Here we automatically run the pa2-debug-a data. And use the nav result from CT through loading pa2-debug-a-output2.txt. The result is in a reasonable range.



## IV. Results and Discussions



