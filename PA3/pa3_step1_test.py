import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
from cispa.Registration import regist_matched_points
import cispa.DataProcess as DP
from cispa.PivotCalibration import calib_pivot_points
import numpy as np
import logging
import click
from rich.logging import RichHandler

'''
Problem Description:
Input the body description data and the readings of the optical trackers
Perform registration to 

Steps taken:

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
    rigidbody_B_tip = rigidbody_B[ Nmarkers : , : ]

    # Input the tracker readings here
    readings_path = data_dir / f"{name}-SampleReadingsTest.txt"
    readings, readings_info = DP.load_txt_data(readings_path)

    Nframes = int(readings_info[1])
    NS = int(readings_info[0])
    ND = NS - 2 * Nmarkers

    # Extract the readings for A and B body
    readings_A_body = []
    readings_B_body = []
    for i in range(Nframes):
        readings_A_body.append( readings[ i * NS : i * NS + Nmarkers, : ])
        readings_B_body.append(readings[ i * NS + Nmarkers : i * NS + 2 * Nmarkers, : ])

    


if __name__ == "__main__":
    main()
