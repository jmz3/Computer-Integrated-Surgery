import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
import click
from cispa.Registration import regist_matched_points
from cispa.PivotCalibration import calib_pivot_points
import cispa.DataProcess as DP
import numpy as np
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

    pivot_path = data_dir / f"{name}-empivot.txt"
    pivot_data,pivot_info = DP.load_txt_data(pivot_path)

    if not output_dir.exists():
        output_dir.mkdir()

    NFrames = int(pivot_info[1])
    NG = int(pivot_info[0])

    p_t = np.array([[1],[0.5],[-2]])
    p_t = np.reshape(p_t,(3,1))
    p_pivot = np.array([[9.8],[10.2],[3.4]])
    p_pivot = np.reshape(p_pivot,(3,1))
    log.debug(f"Suppose we have a probe\np_t = {p_t}\np_pivot={p_pivot}")


    G0 = pivot_data[0:6,:].T
    g,g_mean = DP.centralize(G0)
    F_G = []

    for i in range(NFrames):
        G = pivot_data[NG*i:NG*(i+1),:].T
        F_G.append(regist_matched_points(g,G))
    
    F_test = []
    for i in range(NFrames):
        Ri = F_G[i][0:3,0:3]
        pi = np.reshape(p_pivot - Ri @ p_t,(3,1))
        F_i = np.hstack((Ri,pi))
        F_i = np.vstack((F_i,np.ones((1,4))))
        F_test.append(F_i)
    
    p_t_cal, p_pivot_cal = calib_pivot_points(F_test)
    log.debug(f"The calibration result \np_t = {p_t_cal} \np_pivot={p_pivot_cal}")

    if np.allclose(p_pivot_cal - p_pivot, np.zeros((3,1))):
        log.info(f"Calibration result is correct!")

if __name__ == "__main__":
    main()

