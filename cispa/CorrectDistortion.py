from math import comb
import numpy as np
from scipy.special import comb


def Scale2Box(q):

    '''
    Data must come as a "row vector" i.e. [obj1, obj2, ..., objn]    
    where obj is often given as [ x ; y ; z ]
    '''
    # if q is not in the required shape, transpose it
    if q.shape[1] == 3:
        q = q.T

    point_dim = q.shape[0]
    point_count = q.shape[1]

    min = np.min(q,axis=1,keepdims=True)
    max = np.max(q,axis=1,keepdims=True)

    for i in range(point_count):
        # print(i)
        for j in range(point_dim):
            q[j,i] = (q[j,i] - min[j]) / (max[j]-min[j])


def bernstein(q,order: int=5):
    '''
    q is the input point set

    Data must come as a "row vector" i.e. [obj1, obj2, ..., objn]    
    where obj is often given as [ x ; y ; z ]

    Bernstein basis is consisted of the binary combination of (1-v) and (v)
    
    Bernstein polynomial is consisted of the product of B(x), B(y), B(z)
    
    Tensor T ( aka matrix F in Handouts ) is consisted of bernstein polynomials 
    each entry of that tensor is a row vector 
    which is : [F000(pi), F001(pi), ..., Fijk(pi), ..., F555(pi)]
    Fijk is just a scalar for given pi=[ xi ; yi ; zi ]
    '''
    bern_basis = lambda i,v: comb(order, i) * pow(1 - v, order - i) * pow(v, i)
    bern_poly = lambda i,j,k,u: bern_basis(i,u[0]) * bern_basis(j,u[1]) * bern_basis(k,u[2])

    Scale2Box(q) # Scale q to a box of [0,1]*[0,1]*[0,1]

    T = np.zeros((q.shape[1],pow(order+1,3))) # The tensor should have dim of (NUM x 6^3 )

    NUM = q.shape[1] # NUM is the number of input data points 
    col = 0 # col is the column index of the Tensor T

    for i in range(order+1):
        for j in range(order+1):
            for k in range(order+1):
                # compute for all points at ijk
                T[:,col] = [bern_poly(i,j,k,q[:,num]) for num in range(NUM)]
                col += 1
    
    return T

def calc_coeff(T, p):
    return 0


if __name__=="__main__":
    q = np.array([[1.0,20,3],[7,5,6],[2,0,5],[10,9,8]])
    ground_truth = np.array([[0.9,20.1,2.9],[7,5,6],[2.1,0.1,5.0],[10.1,9.3,8.5]])
    q = q.T
    print(q)
    Scale2Box(q)
    T = bernstein(q)
    # coeff = np.linalg.lstsq(T,ground_truth.T, rcond=None)[0]
    print(ground_truth.shape[0])