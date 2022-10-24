import numpy as np

def load_txt_data(file_name):
    data = np.loadtxt(file_name,skiprows=1,delimiter=',')
    data_info = np.loadtxt(file_name,max_rows=1,dtype=str,delimiter=',')
    return data,data_info

def save_txt_data(output_path, Title, Output):
    with open(output_path,"w") as f:
        np.savetxt(f, Title, delimiter=', ',fmt='%s')
        np.savetxt(f, Output, delimiter=' ', fmt='%10.5f')


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

def dehomovec(x):
    # pull out the 3x1 vector from a homogeneous set
    if x.shape[0] == 4: return np.delete(x,3,0)
    elif x.shape[1] == 4: return np.delete(x,3,1)
    else:
        print("Wrong Shape! Please check your input!")
        return x
