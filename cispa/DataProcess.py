import numpy as np

def load_txt_data(file_name):
    data = np.loadtxt(file_name,skiprows=1,delimiter=',')
    data_info = np.loadtxt(file_name,max_rows=1,dtype=str,delimiter=',')
    return data,data_info

def load_txt_modes(file_name):
    mode_type = []
    mode_data = []
    mode_data_temp = []
    with open(file_name) as f:
        for line in f:
            if line.startswith('Problem'):
                header = line.split()
            elif line.startswith('Mode'):
                mode_type.append(line)
            else:
                mode_data_temp.append(line.removesuffix('\n'))
    mode_info = [header[1].split('='),header[2].split('=')]

    for line in mode_data_temp:
        line = line.split(',')
        mode_data.append(line)
    mode_data = np.array(mode_data).astype(np.float)
    return mode_data, mode_info, mode_type           

    # return data,data_info

def load_txt_data_with_space(file_name):
    data = np.loadtxt(file_name,skiprows=1,dtype=str)
    data = data.astype(np.float)
    data_info = np.loadtxt(file_name,max_rows=1,dtype=str,delimiter=' ')
    return data,data_info

def load_mesh_data(file_name):
    Nvertex = int(np.loadtxt(file_name,max_rows=1,delimiter=','))
    vertex = np.loadtxt(file_name,skiprows=1,max_rows=Nvertex,delimiter=' ')
    
    Nfaces = int(np.loadtxt(file_name,skiprows=Nvertex+1,max_rows=1,delimiter=','))
    face_idx = np.loadtxt(file_name,skiprows=Nvertex+2,max_rows=Nfaces,delimiter=' ')
    return Nvertex, vertex, Nfaces, face_idx

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
