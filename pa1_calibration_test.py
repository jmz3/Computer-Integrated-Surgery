from importlib.resources import path
from pathlib import Path
from cispa.Registration import regist_matched_points
import cispa.DataProcess as DP
from cispa.PivotCalibration import calib_pivot_points
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

    pivot_path = data_dir / f"{name}-empivot.txt"
    pivot_data,pivot_info = DP.load_txt_data(pivot_path)

    Group_Num = int(pivot_info[1])
    Group_Size = int(pivot_info[0])

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

    for i in range(Group_Num):
        G = pivot_data[Group_Size*i:Group_Size*(i+1),:].T
        F_G.append(regist_matched_points(g,G))
    
    print(F_G[1])


