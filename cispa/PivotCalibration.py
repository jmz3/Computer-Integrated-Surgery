import numpy as np
from scipy.linalg import lstsq
from typing import List, Tuple



def calib_pivot_points( frames: List[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:

   
    """Compute the pivot calibration of a robot arm.

    Args:
        frames: A list of FrameTransforms, each of which represents the
            transformation from the robot's base frame to the tool frame
            at a given time.

    Returns:
        A tuple of two numpy.ndarrays, representing the tip_in_tool and
        post_in_world transformations.
    """
    # Your implementation
    Data_Num = len(frames)

    RI = np.zeros((3*Data_Num,6)) # Matrix of (Ri - I)
    NP = np.zeros((3*Data_Num,1)) # Vector of -Pi
    for i in range(Data_Num):
        RI[3*i:3*i+3,:] = np.hstack(((frames[i])[:3,:3],- np.eye(3)))
        NP[3*i:3*i+3,0] = - frames[i][:3,3]

    # solve least square problem for RI*p = NP
    p = lstsq(RI,NP)[0]
    tip_in_tool = p[0:3]
    post_in_world = p[3:6]

    # print(p)
    return tip_in_tool, post_in_world
