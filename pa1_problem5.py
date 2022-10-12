from importlib.resources import path
from pathlib import Path
from ProgrammingAssignment.PA1_workspace.pa1_problem6 import NFrames
from cispa.Registration import regist_matched_points
import cispa.DataProcess as DP
from cispa.PivotCalibration import calib_pivot_points
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

    pivot_path = data_dir / f"{name}-empivot.txt"
    pivot_data,pivot_info = DP.load_txt_data(pivot_path)

    NFrames = int(pivot_info[1])
    NG = int(pivot_info[0])

    if not output_dir.exists():
        output_dir.mkdir()

    logging.basicConfig(
        level="DEBUG",
        format="%(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(rich_tracebacks=True)],
    )

    log = logging.getLogger(__name__)

    G0 = pivot_data[0:6,:].T
    g,g_mean = DP.centralize(G0)
    F_G = []

    for i in range(NFrames):
        G = pivot_data[NG*i:NG*(i+1),:].T
        F_G.append(regist_matched_points(g,G))
    
    p_t, p_pivot = calib_pivot_points(F_G)

    log.info(f"Pt = \n{p_t} \nPpivot = \n{p_pivot}")


