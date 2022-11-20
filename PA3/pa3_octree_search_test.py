import os
import sys
sys.path.append(os.path.dirname(sys.path[0]))
import logging
from pathlib import Path

import click
import numpy as np
from matplotlib import pyplot as plt
from rich.logging import RichHandler

import cispa.DataProcess as DP
from cispa.CarteFrame import CarteFrame
from cispa.FindClosestPoint2Mesh import FindClosestPoint2Mesh
from cispa.Octree import Octree
from cispa.PivotCalibration import calib_pivot_points
from cispa.Registration import regist_matched_points

'''
Problem Description:
Find the closest point on the mesh to a given point using the octree data structure

Steps taken:
1. Load the mesh data
2. Generate a random point
3. Find the closest point on the mesh to the random point using the octree data structure
4. Plot the mesh and the random point to check if it is correct

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
    face_idx = face_idx[:, :3].astype(int) # type conversion to int
    face_idx = face_idx # index starts from 0

    Closest2Mesh_ = FindClosestPoint2Mesh(vertex, Nface, face_idx) # initialize an object of FindClosestPoint2Mesh
    point = np.array([20.0,-30.0,0.0]).reshape(3)

    closest_point = Closest2Mesh_.OctreeSolver(point)
    print(closest_point)

    fig = plt.figure(figsize=(10,10))
    plt.set_loglevel('warning')
    ax = fig.add_subplot(projection='3d')
    ax.scatter(vertex[:,0],vertex[:,1],vertex[:,2],marker='.')
    ax.scatter(closest_point[0],closest_point[1],closest_point[2],marker='o',color='r')
    ax.scatter(point[0],point[1],point[2],marker='o',color='r')
    offset = 1
    ax.text(closest_point[0]+offset,closest_point[1]+offset,closest_point[2]+offset,"closest point")
    ax.text(point[0]+offset,point[1]+offset,point[2]+offset,"tip point")

    plt.show()
    
    

if __name__ == "__main__":
    main()
