from pathlib import Path
from cispa.Registration import regist_matched_points
from cispa.PivotCalibration import calib_pivot_points
import cispa.DataProcess as DP
import numpy as np
from scipy.spatial.transform import Rotation as ROT
import logging
from rich.logging import RichHandler

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

    d = cal_body[0:8,:].T
    D = cal_read[0:8,:].T
    F_d = regist_matched_points(d,D)
    F_dinv = np.linalg.inv(F_d)

    opt_pivot_path = data_dir / f"{name}-optpivot.txt"
    opt_pivot_data,opt_pivot_info = DP.load_txt_data(opt_pivot_path)

    NFrames = int(opt_pivot_info[2])
    NH = int(opt_pivot_info[1])
    ND = int(opt_pivot_info[0])

    D0 = F_dinv @ DP.homovec(opt_pivot_data[ND:ND+NH,:].T)
    D0 = DP.dehomovec(D0)
    d,d_bar = DP.centralize(D0)
    F = []

    for i in range(NFrames):

        D = F_dinv @ DP.homovec(opt_pivot_data[(ND+NH)*i+ND:(NH+ND)*(i+1),:].T)
        D = DP.dehomovec(D)
        F.append(regist_matched_points(d,D))

    # print(F[1])

    p_t, p_pivot = calib_pivot_points(F)

    log.info(f"Pt = \n{p_t} \nPpivot = \n{p_pivot}")



