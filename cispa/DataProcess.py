import numpy as np

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

def centralize(x):

    x_bar = x - np.repeat(np.reshape(np.mean(x,1),(3,1)),x.shape[1],axis=1)
    x_mean = np.reshape(np.mean(x,1),(3,1))

    return x_bar, x_mean

def homovec(x):
    # return the homogeneous form of vector x
        if x.shape[0] == 3:
            return np.vstack((x,np.ones((1,x.shape[1]))))
        
        elif x.shape[1] == 3:
            return np.hstack((x,np.ones((x.shape[0],1))))
        
        else:
            print("Wrong Shape! Please check your input!")
            return x        

