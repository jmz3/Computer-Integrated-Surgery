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
from cispa.CarteFrame import CarteFrame

'''
Problem Description:
Apply the distortion correction to all the subsequent values of [G1, ... , Gn]
Compute the pointer tip coordinates w.r.t. the tracker base
And apply Freg to compute the tip location w.r.t. the CT image.


steps taken:
1. Same steps as in PA2/pa2_problem5_test.py
2. Apply the distortion correction to the point cloud in EM_nav
3. Register the point cloud in EM_nav to the point cloud in em_pivot to find the registration frame Fem
4. Derive the position of the pointer tip in the CT frame through p_pivot = Fem * p_tip p_ct = Freg * p_pivot 
5. In general, the probe point position in CT C.S. can be determined through p_ct = Freg * Fem * p_tip
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
    output1_path = output_dir / f"{name}-output1.txt"
    output2_path = output_dir / f"{name}-output2.txt"

    ###########################################################################
    ########### The process is the same as PA2/pa2_problem5_test.py ###########
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

    # load the given point set: EM fiducial points
    em_fiducial_path = data_dir / f"{name}-em-fiducialss.txt"
    em_fiducial_data,em_fiducial_info = DP.load_txt_data(em_fiducial_path)
    NG = int(em_fiducial_info[0])
    NFrames = int(em_fiducial_info[1])

    # correct the distortion
    em_fiducial_corrected = CorrectDistortion.predict(em_fiducial_data, correction_coeff)

    # perform point cloud registration
    p_fiducial = []
    F_G = np.zeros((4,4))
    for k in range(NFrames):
        G = em_fiducial_corrected[NG*k:NG*(k+1),:].T
        F_G = regist_matched_points(g,G)
        F_Gk = CarteFrame(F_G[0:3,0:3], F_G[0:3,3])
        p_fiducial.append(F_Gk @ p_t)
    
    b = np.array(p_fiducial, dtype=np.float64, order='C').reshape(-1,3)
    b = b.T

    ct_fiducial_path = data_dir / f"{name}-ct-fiducials.txt"
    ct_fiducial_data,ct_fiducial_info = DP.load_txt_data(ct_fiducial_path)
    NB = int(ct_fiducial_info[0])

    Freg = regist_matched_points(b, ct_fiducial_data.T)
    

    ###########################################################################
    ################### Locate the Probe w.r.t. CT image ######################
    ###########################################################################
    # load the given point set: EM NAV points
    em_nav_path = data_dir / f"{name}-EM-nav.txt"
    em_nav_data,em_nav_info = DP.load_txt_data(em_nav_path)
    NG = int(em_nav_info[0])
    NFrames = int(em_nav_info[1])

    # correct the distortion
    em_nav_corrected = CorrectDistortion.predict(em_nav_data, correction_coeff)
    p_ct = []
    for k in range(NFrames):
        G = em_nav_corrected[ NG * k : NG * ( k + 1 ) , : ].T
        Fem = regist_matched_points( g , G )
        F = Freg @ Fem
        Fk = CarteFrame(F[0:3,0:3], F[0:3,3])
        p_ct.append( Fk @ p_t )

    p_ct = np.array(p_ct, dtype=np.float64, order='C').reshape(-1,3)
    log.info(f"p_ct = \n{p_ct}")

    
    # load the output2 data to compare
    if 'debug' in name:
        result_path = data_dir / f"{name}-output2.txt"
        result_data,result_info = DP.load_txt_data(result_path)
        log.info(f"debug data = \n{result_data}")
        log.info(f"Total error = {np.mean(np.linalg.norm(p_ct - result_data, axis=1))}")

    # save the result
    output_path = output_dir / f"{name}-own-output2.txt"
    Title = Title = np.array([[f'{NFrames}',f"{name}-own-output2.txt"]],dtype=str)
    Output = p_ct
    DP.save_txt_data(output_path,Title,Output)

if __name__=="__main__":
    main()
