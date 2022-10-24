from .DataProcess import load_txt_data
from .Registration import regist_matched_points
from pathlib import Path
import numpy as np
from numpy.linalg import inv

def C_expected(calbody_path, reading_path):
    '''
    param:
    ---------------------------------------------------------------------------
    calbody_path: Path object , values = calbody data directory
    reading_path: Path object , values = calreadings data directory


    return:
    ---------------------------------------------------------------------------
    c_expected: List object, every element of the list is a (3xNC) matrix
                The total lenght of the list is Nframes

    '''
    calbody,calbody_info = load_txt_data(calbody_path)
    reading,reading_info = load_txt_data(reading_path)

    ND = int(calbody_info[0])
    NA = int(calbody_info[1])
    NC = int(calbody_info[2])
    Ntotal = ND+NA+NC
    Nframes = int(reading_info[3])

    # extract d, a and c from calbody data
    d = calbody[0:ND,:].T
    a = calbody[ND:ND+NA,:].T
    c = calbody[ND+NA:,:].T
    c = np.vstack((c,np.ones((1,np.size(c,axis=1))))) # expand c to homogenous form

    # find transformation from D in Kth frame (D[k]) to d
    # find transformation from A in Kth frame (D[k]) to a
    c_expected = []
    c_readings = []
    for k in range(Nframes):
        Dk = reading[0 + k*Ntotal : ND + k*Ntotal , : ].T
        Fd = regist_matched_points(d,Dk)

        Ak = reading[ND + k*Ntotal : ND + NA + k*Ntotal, : ].T
        Fa = regist_matched_points(a,Ak)

        Ck = reading[ND + NA + k*Ntotal : ND + NA + NC + k*Ntotal , : ].T
        # compute c_expected for kth frame and put them in a list
        # c_temp is a (3xNC) Matrix 
        c_temp = (inv(Fd) @ Fa @ c)[0:3,:]
        # c_expected is a list with Nframes elements
        c_expected.append(c_temp)
        # c_readings is a list of Ck in the calreadings data
        c_readings.append(Ck)
    
    return c_expected, c_readings

if __name__=="__main__":
    print()