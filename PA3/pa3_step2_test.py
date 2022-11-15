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
    mesh_path = data_dir / f"Problem3MeshFile.sur"

    result_path = data_dir / f"{name}-output1.txt"
    output_path = output_dir / f"{name}-own-output1.txt"

    Nvertex, vertex, Nface, face_idx = DP.load_mesh_data(mesh_path)
    face_idx = face_idx[:, :3]
    vertex_idx = face_idx[0,:].astype(int) # type conversion to int
    vertex_idx = vertex_idx - 1 # subtract 1 to make it 0-indexed
    triangle = vertex[vertex_idx,:]
    print(triangle)

if __name__ == "__main__":
    main()
