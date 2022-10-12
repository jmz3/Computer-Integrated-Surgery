# PA1 REPORT

\pagebreak
## Mathematical Approaches
### 1. Registration

A point-cloud to point-cloud registration is implemented in this section.

#### 1.1. Method
Our implementation is refferred to Arun's Method. The paper titled “Least-Squares Fitting of Two 3-D Point Sets”, published by A. S. Arun, T. S. Huang and S. D. Blostein on 1987, describes a non-iterative least-squares approach to match two sets of 3D points.
Suppose we have two point sets and denote them as $ P $ and $ P' $, and they are representing the same rigid body described at different poses. Hence, they should follows:
$$ P' = RP+t $$
