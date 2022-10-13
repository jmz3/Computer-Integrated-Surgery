import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
import click
from cispa.Registration import regist_matched_points
from cispa.PivotCalibration import calib_pivot_points
import cispa.DataProcess as DP
import numpy as np
from scipy.spatial.transform import Rotation as ROT
import logging
from rich.logging import RichHandler
from numpy.linalg import inv

logging.basicConfig(
    level="DEBUG",
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
)
log = logging.getLogger(__name__)

@click.command()
@click.option("--data-dir", "-d", default="PA1/Data", help="Input data directory")
@click.option("--output-dir", "-o", default="PA1/output", help="Output directory")
@click.option("--name", "-n", default="pa1-debug-a", help="Name of the output file")
def main(data_dir, output_dir, name):
    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()

    cal_path = data_dir / f"{name}-calbody.txt"
    cal_body,calbody_info = DP.load_txt_data(cal_path)

    r = ROT.from_euler('z', 180, degrees=True)
    R = r.as_matrix()

    log.debug(f"Suppose we apply a transformation\nR = {R}\np={10*np.ones((3,1))}")
    cal_body = cal_body[0:8,:].T
    cal_goal = R @ cal_body + np.repeat(10*np.ones((3,1)),8,axis=1)

    F = regist_matched_points(cal_body,cal_goal)
    log.debug(f"The registered transformation \nR = {F[0:3,0:3]} \np={F[0:3,3]}")
    R_regist, p_regist = F[0:3,0:3], F[0:3,3]

    if np.allclose(R @ inv(R_regist), np.eye(3)) and np.linalg.det(R_regist) == 1.0:
        log.info(f"Registered rotation matrix is correct")

    if np.allclose(10*np.ones((3,1)) - p_regist, np.zeros((3,1))):
        log.info(f"Registered translation vector is correct")

if __name__ == "__main__":
    main()

