import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
import numpy as np
import logging
import click
from rich.logging import RichHandler
import cispa.DataProcess as DP
from cispa import CorrectDistortion
from cispa import ComputeExpectValue
from cispa.Registration import regist_matched_points
from cispa.PivotCalibration import calib_pivot_points

'''
Problem Description:
Using the distortion correction and the improved pivot value, 
compute the locations of the fiducials points 
w.r.t. the EM tracker base coordinate system

steps taken:
1.  Same steps as in PA2/pa2_problem3_test.py
2.  Get EM-fiducial point cloud and correct the distortion
3.  Perform point cloud registration between EM-fiducial points and gmean
    gmean is the average point for G[0] in the pivot calibration process
4.  Find the p_pivot for every frame of EM-fiducial points according to the formula:
    p_pivot = F*p_tip + gmean
'''

logging.basicConfig(
        level="DEBUG",
        format="%(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(rich_tracebacks=True)],
    )

log = logging.getLogger(__name__)


@click.command()
@click.option("--data-dir", "-d", default="PA2/Data", help="Input data directory")
@click.option("--output-dir", "-o", default="PA2/output", help="Output directory")
@click.option("--name", "-n", default="pa2-debug-a", help="Name of the output file")
def main(data_dir, output_dir, name):
    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()
    if not output_dir.exists():
        output_dir.mkdir()
    
    cal_body_path = data_dir / f"{name}-calbody.txt"
    cal_read_path = data_dir / f"{name}-calreadings.txt"
    output_path = output_dir / f"{name}-bernstein-coeff.txt"

    ###########################################################################
    ########### The process is the same as PA2/pa2_problem3_test.py ###########
    ###########################################################################
    # Correct the distortion
    c_expected, c_readings = ComputeExpectValue.C_expected(cal_body_path,cal_read_path)
    ground_truth = np.concatenate([c_expected[i].T for i in range(len(c_expected))])
    sensor_data = np.concatenate([c_readings[i].T for i in range(len(c_readings))])
    correction_coeff = CorrectDistortion.fit(sensor_data, ground_truth)

    # Perform pivot calibration
    pivot_path = data_dir / f"{name}-empivot.txt"
    pivot_data,pivot_info = DP.load_txt_data(pivot_path)
    NFrames = int(pivot_info[1])
    NG = int(pivot_info[0])
    pivot_corrected = CorrectDistortion.predict(pivot_data, correction_coeff)
    
    # regular pivot calibration on corrected data
    G0 = pivot_corrected[0:NG,:].T
    g,g_mean = DP.centralize(G0)
    F_G = []
    for i in range(NFrames):
        G = pivot_corrected[NG*i:NG*(i+1),:].T
        F_G.append(regist_matched_points(g,G))
    
    p_t, p_pivot = calib_pivot_points(F_G)

    ###########################################################################
    ######## Now find the p_pivot at frame k based on given point set #########
    ###########################################################################
    # load the given point set: EM fiducial points
    em_fiducial_path = data_dir / f"{name}-em-fiducialss.txt"
    em_fiducial_data,em_fiducial_info = DP.load_txt_data(em_fiducial_path)
    NG = int(em_fiducial_info[0])
    NFrames = int(em_fiducial_info[1])



    

if __name__=="__main__":
    main()