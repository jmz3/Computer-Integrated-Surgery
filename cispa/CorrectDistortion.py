from math import comb
import numpy as np
from scipy.special import comb
# from .DataProcess import save_txt_data
from pathlib import Path


def Scale2Box(q, ReturnBound=False):

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

    qs = np.zeros_like(q)
    for i in range(point_count):
        # print(i)
        for j in range(point_dim):
            qs[j,i] = (q[j,i] - min[j]) / (max[j]-min[j])
        
        assert np.max(qs[:,i]) <=1 and np.min(qs[:,i]) >= 0

    if ReturnBound == False:
        return qs
    else:
        return qs, min, max

def bernstein(q,order: int=5):
    '''
    param:
    ---------------------------------------------------------------------------
    q: the input point set
    order: the highest order of bernstein polynomials

    return:
    ---------------------------------------------------------------------------
    T: Bernstein tensor for all points in q

    Data must come as a "row vector" i.e. [obj1, obj2, ..., objn]    
    where obj is often given as [ x ; y ; z ]

    Bernstein basis is consisted of the binary combination of (1-v) and (v)
    
    Bernstein polynomial is consisted of the product of B(x), B(y), B(z)
    
    Tensor T ( aka matrix F in Handouts ) is consisted of bernstein polynomials 
    each entry of that tensor is a row vector 
    which is : [F000(pi), F001(pi), ..., Fijk(pi), ..., F555(pi)]
    Fijk is just a scalar for given pi=[ xi ; yi ; zi ]
    '''
    if q.shape[1] == 3:
        q = q.T
    bern_basis = lambda i,v: comb(order, i) * pow(1 - v, order - i) * pow(v, i)
    # bern_basis = lambda i,v: np.asarray(np.math.factorial(i),np.float32)/np.math.factorial(order)* v**i * (1-v)**(order-i)
    bern_poly = lambda i,j,k,u: bern_basis(i,u[0]) * bern_basis(j,u[1]) * bern_basis(k,u[2])

    qs = Scale2Box(q) # Scale q to a box of [0,1]*[0,1]*[0,1]

    T = np.zeros((qs.shape[1],pow(order+1,3))) # The tensor should have dim of (NUM x 6^3 )

    NUM = qs.shape[1] # NUM is the number of input data points 
    col = 0 # col is the column index of the Tensor T

    for i in range(order+1):
        for j in range(order+1):
            for k in range(order+1):
                # compute for all points at ijk
                T[:,col] = [bern_poly(i,j,k,qs[:,num]) for num in range(NUM)]
                col += 1
    
    return T

def fit(q, p):
    '''
    param:
    ---------------------------------------------------------------------------
    q: the input "distorted" data
    p: the ground truth 

    return:
    ---------------------------------------------------------------------------
    coeff:  the matrix of coefficiences for bernstein polynomials
            it can also be considered as 
    '''

    # check the dimension of q and convert it into 3xN form
    if q.shape[1] == 3:
        q = q.T

    # convert p into a set of row vectors, 
    # i.e. q = [   [x1,y1,z1]
    #              [x2,y2,z2]
    #              ...         ]   
    if p.shape[0] == 3:
        p = p.T

    # calculate the bernstein tensor
    # print("I'm here")
    T = bernstein(q, order=5)

    # the groundtruth also needs to be scaled to BOX
    ps = Scale2Box(p).T
    coeff = np.linalg.lstsq(T,ps, rcond=None)[0]
    
    return coeff

def predict(q, coeff):
    '''
    param:
    ---------------------------------------------------------------------------
    q: the input "distorted" data
    coeff:  the "fitted" coefficient matrix that is trained through 'fit' function

    return:
    ---------------------------------------------------------------------------
    q_corrected: data that has been corrected by distortion correction function
    '''

    # find the scaled q_corrected data
    q_corrected = bernstein(q, order=5) @ coeff

    # descale: zoom the data to its original range
    qs, MIN, MAX = Scale2Box(q, ReturnBound=True)
    if qs.shape[0] == 3:
        qs = qs.T
    for i in range(q_corrected.shape[1]):
        for j in range(q_corrected.shape[0]):
            q_corrected[j,i] = qs[j,i]*(MAX[i]-MIN[i]) + MIN[i]


    return q_corrected


if __name__=="__main__":
    # q = np.array([[1.0,20,3],[7,5,6],[2,0,5],[10,9,8]])
    # ground_truth = np.array([[0.9,20.1,2.9],[7,5,6],[2.1,0.1,5.0],[10.1,9.3,8.5]])
    # q = q.T
    # print(q)
    # Scale2Box(q)
    # print(fit(q,ground_truth))

    # data_dir="PA2/Data"
    # output_dir="PA2/output"
    # name="pa2-debug-a"
    # data_dir = Path(data_dir).expanduser()
    # output_dir = Path(output_dir).expanduser()
    # if not output_dir.exists():
    #     output_dir.mkdir()

    # cal_body_path = data_dir / f"{name}-calbody.txt"
    # cal_read_path = data_dir / f"{name}-calreadings.txt"
    # output_path = output_dir / f"{name}-bernstein-coeff.txt"
    

    # T = []
    # Title = np.array([[f"{name}-bernstein-coeff.txt"]],dtype=str)
    # save_txt_data(output_path, Title, T)
    i = 1
    order = 5
    x = np.asarray(np.math.factorial(i),np.float32)/np.math.factorial(order)
    y = comb(order,i)
    print(np.asarray(np.math.factorial(i),np.float32))
    print(np.linspace(1,100,4))
    print(x)
    print(y)