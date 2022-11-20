import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
from cispa.CarteFrame import CarteFrame
from cispa.Registration import regist_matched_points
import cispa.DataProcess as DP
from cispa.PivotCalibration import calib_pivot_points
import numpy as np
import logging
import click
from rich.logging import RichHandler

'''
Problem Description:
Find the tool tip position dk w.r.t. the fiducial coordinate system 

Steps taken:
1. Load rigid body description
2. Load the readings of the LED trackers
3. Perform point cloud registration to find Fak, Fbk
4. dk = inv(Fbk)*Fak*Atip

'''

logging.basicConfig(
    level="DEBUG",
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
)

log = logging.getLogger(__name__)


@click.command()
@click.option("--data-dir", "-d", default="PA3/Data", help="Input data directory")
@click.option("--output-dir", "-o", default="PA3/output", help="Output directory")
@click.option("--name", "-n", default="PA3-A-Debug", help="Name of the output file")
def main(data_dir, output_dir, name):
    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()
    if not output_dir.exists():
        output_dir.mkdir()

    # Input the body description here
    rigidbody_A_path = data_dir / f"Problem3-BodyA.txt"
    rigidbody_B_path = data_dir / f"Problem3-BodyB.txt"

    result_path = data_dir / f"{name}-output1.txt"
    output_path = output_dir / f"{name}-own-output1.txt"

    rigidbody_A, rigidbody_A_info = DP.load_txt_data_with_space(
        rigidbody_A_path)
    rigidbody_B, rigidbody_B_info = DP.load_txt_data_with_space(
        rigidbody_B_path)

    # Extract the information of the input data
    Nmarkers = int(rigidbody_A_info[0])

    rigidbody_A_body = rigidbody_A[ 0 : Nmarkers , : ]
    rigidbody_A_tip = rigidbody_A[ Nmarkers : , : ]
    rigidbody_B_body = rigidbody_B[ 0 : Nmarkers , : ]

    # Input the tracker readings here
    readings_path = data_dir / f"{name}-SampleReadingsTest.txt"
    readings, readings_info = DP.load_txt_data(readings_path)

    Nframes = int(readings_info[1])
    NS = int(readings_info[0])
    print(rigidbody_A_tip.shape)
    # Extract the readings for A and B body
    # Perform registration on the readings
    d_tip = []
    for i in range(Nframes):
        readings_A_body = readings[ i * NS : i * NS + Nmarkers, : ]
        F_A = regist_matched_points(rigidbody_A_body, readings_A_body)
        F_Ak = CarteFrame(F_A[0:3,0:3], F_A[0:3,3])

        readings_B_body = readings[ i * NS + Nmarkers : i * NS + 2 * Nmarkers, : ]
        F_B = regist_matched_points(rigidbody_B_body, readings_B_body)
        F_Bk = CarteFrame(F_B[0:3,0:3], F_B[0:3,3])

        # Compute pointer tip w.r.t B body
        F_Bk.inverse()
        d_tip.append(F_Bk @ F_Ak @ rigidbody_A_tip)

    d_tip = np.concatenate(d_tip, axis = 1).T
    log.info(f"dk for {name} data: \n{d_tip}")


if __name__ == "__main__":
    main()
