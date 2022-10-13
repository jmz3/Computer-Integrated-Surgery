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

    cal_body_path = data_dir / f"{name}-calbody.txt"
    cal_body,cal_body_info = DP.load_txt_data(cal_body_path)

    cal_read_path = data_dir / f"{name}-calreadings.txt"
    cal_read,cal_read_info = DP.load_txt_data(cal_read_path)

    pivot_path = data_dir / f"{name}-empivot.txt"
    pivot_data,pivot_info = DP.load_txt_data(pivot_path)

    opt_pivot_path = data_dir / f"{name}-optpivot.txt"
    opt_pivot_data,opt_pivot_info = DP.load_txt_data(opt_pivot_path)

    output_path = output_dir / f"{name}-output.txt"


    """
    Registration Part
    """
    ND = int(cal_body_info[0])
    NA = int(cal_body_info[1])
    # Register about EM Trackers
    d = cal_body[0:ND,:].T
    D = cal_read[0:ND,:].T
    F_d = regist_matched_points(d,D)

    # log.info(f"F_d = {F_d}")

    # Register about Optical Trackers
    a = cal_body[ND:ND+NA,:].T
    A = cal_read[ND:ND+NA,:].T
    F_a = regist_matched_points(a,A)

    # log.info(f"F_a = {F_a}")

    # Find expected C
    c = cal_body[ND+NA:,:].T
    c = np.vstack((c,np.ones((1,np.size(c,axis=1)))))
    C_expected = inv(F_d) @ F_a @ c
    
    """
    Pivot calibration on EM probe
    """

    NFrames = int(pivot_info[1])
    NG = int(pivot_info[0])

    G0 = pivot_data[0:6,:].T
    g,g_mean = DP.centralize(G0)
    F_G = []

    for i in range(NFrames):
        G = pivot_data[NG*i:NG*(i+1),:].T
        F_G.append(regist_matched_points(g,G))
    
    p_t_EM, p_pivot_EM = calib_pivot_points(F_G)

    """
    Pivot calibration on Optical probe
    """
    F_dinv = np.linalg.inv(F_d)

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

    p_t_op, p_pivot_op = calib_pivot_points(F)

    """
    Output
    """

    Output = np.vstack((p_pivot_EM.reshape((1,3)),p_pivot_op.reshape((1,3))))
    Output = np.vstack((Output,C_expected.T))

    np.savetxt('test.out', Output, delimiter=' ')   # X is an array