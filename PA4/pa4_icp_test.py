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
from cispa.IterClosestPoint import ICP

'''
Problem Description:
Input the surface mesh and the readings of the optical trackers
Return the registered points and the transformation matrix

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
@click.option("--data-dir", "-d", default="PA4/Data", help="Input data directory")
@click.option("--output-dir", "-o", default="PA4/output", help="Output directory")
@click.option("--name", "-n", default="PA4-A-Debug", help="Name of the output file")
def main(data_dir, output_dir, name):
    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()
    if not output_dir.exists():
        output_dir.mkdir()

    # Input the body description here
    mesh_path = data_dir / f"Problem4MeshFile.sur"

    output_path = output_dir / f"{name}-own-output.txt"

    Nvertex, vertex, Nface, face_idx = DP.load_mesh_data(mesh_path)
    face_idx = face_idx[:, :3].astype(int) # type conversion to int
    mesh = {}
    mesh['vertex'] = vertex
    mesh['Nface'] = Nface
    mesh['face_idx'] = face_idx




    ############################################################################
    ####################### Compute the dk here ################################
    ############################################################################
    # Input the body description here
    rigidbody_A_path = data_dir / f"Problem4-BodyA.txt"
    rigidbody_B_path = data_dir / f"Problem4-BodyB.txt"

    result_path = data_dir / f"{name}-output.txt"
    output_path = output_dir / f"{name}-own-output.txt"

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


    ############################################################################
    ############## Perform Iterative Closest Point here ########################
    ############################################################################
    # Initialize the ICP object

    ICP_ = ICP(mesh, threshold=[0.01, 0.01, 0.01], max_iter=100)
    F = ICP_.compute_icp_transform(dk)
    print(F.R)
    # brute_result = []
    # octree_result = []
    # start = time.time()
    # for i in range(len(d_tip)):
    #     brute_result.append(Closest2Mesh_.BruteForceSolver(d_tip[i]))
    # end = time.time()
    # log.info(f"Brute Force Solver time = \n{end - start} seconds")

    # start = time.time()
    # for i in range(len(d_tip)):
    #     octree_result.append(Closest2Mesh_.OctreeSolver(d_tip[i]))
    # end = time.time()
    # log.info(f"Octree Solver time = \n{end - start} seconds")

    # ck_brute = [brute_result[i][:] for i in range(len(brute_result))]
    # ck_octree = [octree_result[i][:] for i in range(len(octree_result))]
    
    # ck = ck_octree
    # ck = np.concatenate(ck,axis = 0) # Concatenate list members 
    # ck = np.reshape(ck,(Nframes, 3)) # Transfer to ndarray


    ############################################################################
    ############### Visualization for dk and the surface mesh ##################
    ############################################################################
    fig = plt.figure()
    plt.set_loglevel('info')
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(dk[:,0],dk[:,1],dk[:,2], c = 'r', marker = 'o')
    ax.scatter(vertex[:,0], vertex[:,1], vertex[:,2], c = 'b', alpha=0.2)
    plt.show()
    # log.info(f"dk for {name} data: {d_tip}")


    # ############################################################################
    # ###################### Output the result ###################################
    # ############################################################################
    # diff = np.zeros(Nframes)
    # for i in range(Nframes):
    #     diff[i] = np.linalg.norm(dk[i,:] - ck[i,:])
    
    # diff = np.reshape(diff,(-1,1))
    # log.info(f"|| dk - ck || = \n {diff}")

    # Title = np.array([[f'{Nframes}',f"{name}-Output.txt"]],dtype=str)
    # OutputData = np.hstack([dk,ck])
    # OutputData = np.hstack((OutputData, diff))

    # DP.save_txt_data(output_path, Title, OutputData)


if __name__=="__main__":
    main()
    # container = {}
    # container['vertex'] = np.array([[1,2,3],[4,5,6],[7,8,9]])
    # container['face_idx'] = np.array([[1,2,3]])
    # print(container)

    # a = np.array([[[1,2,3]],[[4,5,6]],[[7,8,9]]])
    # print(a.shape)
    # A = []
    # for i in a:
    #     A.append(i)
    
    # print(A)
    # Ap = np.asarray(A).reshape((3,3))
    # print(Ap.shape)
    # a = 1
    # b = 2
    # c = a <= b
    # print(c)
    # a = np.array([[1,2,3],[4,5,6],[7,8,9]])
    # for i in a:
    #     print(i)
    