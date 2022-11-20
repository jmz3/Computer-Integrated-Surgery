import os
import sys
sys.path.append(os.path.dirname(sys.path[0]))
import logging
from pathlib import Path

import click
import numpy as np
from matplotlib import pyplot as plt
from rich.logging import RichHandler
import time

import cispa.DataProcess as DP
from cispa.CarteFrame import CarteFrame
from cispa.FindClosestPoint2Mesh import FindClosestPoint2Mesh
from cispa.Octree import Octree
from cispa.Registration import regist_matched_points

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
    face_idx = face_idx[:, :3].astype(int) # type conversion to int
    face_idx = face_idx # index starts from 0


    ############################################################################
    ####################### Compute the dk here ################################
    ############################################################################
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
    # print(rigidbody_A_tip.shape)
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
    dk = d_tip
    # log.info(f"dk for {name} data: {d_tip}")

    
    ############################################################################
    ##################### Find CLosest ck for given dk##########################
    ############################################################################
    Closest2Mesh_ = FindClosestPoint2Mesh(vertex, Nface, face_idx) # initialize an object of FindClosestPoint2Mesh

    brute_result = []
    octree_result = []
    start = time.time()
    for i in range(len(d_tip)):
        brute_result.append(Closest2Mesh_.BruteForceSolver(d_tip[i]))
    end = time.time()
    log.info(f"Brute Force Solver time = \n{end - start} seconds")

    start = time.time()
    for i in range(len(d_tip)):
        octree_result.append(Closest2Mesh_.OctreeSolver(d_tip[i]))
    end = time.time()
    log.info(f"Octree Solver time = \n{end - start} seconds")

    ck_brute = [brute_result[i][:] for i in range(len(brute_result))]
    ck_octree = [octree_result[i][:] for i in range(len(octree_result))]
    
    ck = ck_octree
    ck = np.concatenate(ck,axis = 0) # Concatenate list members 
    ck = np.reshape(ck,(Nframes, 3)) # Transfer to ndarray

    ############################################################################
    ###################### Output the result ###################################
    ############################################################################
    diff = np.zeros(Nframes)
    for i in range(Nframes):
        diff[i] = np.linalg.norm(dk[i,:] - ck[i,:])
    
    diff = np.reshape(diff,(-1,1))
    log.info(f"|| dk - ck || = \n {diff}")

    Title = np.array([[f'{Nframes}',f"{name}-Output.txt"]],dtype=str)
    OutputData = np.hstack([dk,ck])
    OutputData = np.hstack((OutputData, diff))

    DP.save_txt_data(output_path, Title, OutputData)


if __name__=="__main__":
    main()