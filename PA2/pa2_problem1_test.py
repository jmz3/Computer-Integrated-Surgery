import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
import logging
import click
from rich.logging import RichHandler
from cispa import ComputeExpectValue

'''
Problem Description:
Input the body calibration data file and process it to 
determine the values of C_expected[k] corresponding to each C_i[k]
in each “frame” k of data.

Steps taken:
1. Load the Optical Marker data for EM Tracker (Di) and Calibration Object (Ai)
2. Load the EM Marker data for Calibration Object (Ci)
3. Compute Fd and Fa according to the formula Aj = Fa * aj , Dj = Fd * dj
4. Find the c_expected for each frame according to the formula C_expected = Fd^-1 * Fa * ci
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

    c_expected, c_readings = ComputeExpectValue.C_expected(cal_body_path,cal_read_path)
    print(c_expected[1].T)
    # print(len(c_readings))



if __name__=="__main__":
    main()