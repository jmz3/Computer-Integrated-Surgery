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
Use this distortion correction function 
to repeat your “pivot calibration” for the EM probe.

Steps taken:
1. Correct the distortion according to the c_expected and c_readings
2. Correct the EM marker coordinates G using the distortion correction function
3. Perform pivot calibration on the corrected data in the traditional way
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
    ###############  Fit the distortion correction function ###################
    ###########################################################################
    # find expected EM marker position w.r.t EM Tracker Coordinate System
    c_expected, c_readings = ComputeExpectValue.C_expected(cal_body_path,cal_read_path)
    
    # Here c_expected and c_readings are a list of (NCx3) matrix
    # We need to expand them into a big matrix, 
    # i.e. extract every position vector and stack them vertically
    ground_truth = np.concatenate([c_expected[i].T for i in range(len(c_expected))])
    sensor_data = np.concatenate([c_readings[i].T for i in range(len(c_readings))])
    # find the coefficents of distortion correction function
    correction_coeff = CorrectDistortion.fit(sensor_data, ground_truth)




    ###########################################################################
    ###################  Perform pivot calibration  ###########################
    ###########################################################################
    pivot_path = data_dir / f"{name}-empivot.txt"
    pivot_data,pivot_info = DP.load_txt_data(pivot_path)
    NFrames = int(pivot_info[1])
    NG = int(pivot_info[0])

    # The bernstein tensor corresponding to pivot data will be computed
    # and multiply it with the correction coefficient matrix
    pivot_corrected = CorrectDistortion.predict(pivot_data, correction_coeff)
    
    # regular pivot calibration on corrected data
    G0 = pivot_corrected[0:NG,:].T
    g,g_mean = DP.centralize(G0)
    F_G = []
    for i in range(NFrames):
        G = pivot_corrected[NG*i:NG*(i+1),:].T
        F_G.append(regist_matched_points(g,G))
    
    p_t, p_pivot = calib_pivot_points(F_G)
    log.info(f"Pt = \n{p_t} \nPpivot = \n{p_pivot}")

    

if __name__=="__main__":
    main()