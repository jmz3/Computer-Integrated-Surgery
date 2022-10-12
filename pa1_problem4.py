from importlib.resources import path
from pathlib import Path
from os import getcwd
from cispa.Registration import regist_matched_points
from cispa.PivotCalibration import calib_pivot_points
import cispa.DataProcess as DP
import numpy as np
from scipy.spatial.transform import Rotation as ROT
import logging
from rich.logging import RichHandler
from numpy.linalg import inv

if __name__ == "__main__":

    data_dir = "PA1 Student Data"
    output_dir = "output"
    name = "pa1-debug-a"

    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()

    cal_body_path = data_dir / f"{name}-calbody.txt"
    cal_body,calbody_info = DP.load_txt_data(cal_body_path)

    cal_read_path = data_dir / f"{name}-calreadings.txt"
    cal_read,cal_read_info = DP.load_txt_data(cal_read_path)
    # log.info(cal_body)
    # log.info(calbody_info)

    if not output_dir.exists():
        output_dir.mkdir()

    logging.basicConfig(
        level="DEBUG",
        format="%(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(rich_tracebacks=True)],
    )

    log = logging.getLogger(__name__)

    """Test Section

    r = ROT.from_euler('z', 180, degrees=True)
    R = r.as_matrix()
    print(R)
    cal_body = cal_body[0:8,:].T
    cal_goal = R @ cal_body + np.repeat(10*np.ones((3,1)),8,axis=1)
    print(cal_goal)
    print(cal_body)

    """

    # Register about EM Trackers
    d = cal_body[0:8,:].T
    D = cal_read[0:8,:].T
    F_d = regist_matched_points(d,D)

    # log.info(f"F_d = {F_d}")

    # Register about Optical Trackers
    a = cal_body[8:16,:].T
    A = cal_read[8:16,:].T
    F_a = regist_matched_points(a,A)

    # log.info(f"F_a = {F_a}")

    # Find expected C
    c = cal_body[16:,:].T
    c = np.vstack((c,np.ones((1,np.size(c,axis=1)))))
    C_expected = inv(F_d) @ F_a @ c
    
    log.info(f"c_e = {C_expected[:,2]}")
    # cal_test = [F,F,F]
    # p_t, p_post = calib_pivot_points(cal_test)
