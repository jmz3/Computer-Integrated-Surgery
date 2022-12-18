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
from cispa.Registration import regist_matched_points
from cispa.IterClosestPoint import ICP

'''
Problem Description:
Input the surface mesh and the readings of the optical trackers
Return the registered points and the transformation matrix that maps the readings to the surface mesh

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
@click.option("--solver", "-s", default="BruteForce", help="Name of the solver, either BruteForce or Octree")
def main(data_dir, output_dir, name, solver):
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
        F_Ak = CarteFrame(F_A[0:3,0:3], F_A[0:3,3].reshape(3,1))

        readings_B_body = readings[ i * NS + Nmarkers : i * NS + 2 * Nmarkers, : ]
        F_B = regist_matched_points(rigidbody_B_body, readings_B_body)
        F_Bk = CarteFrame(F_B[0:3,0:3], F_B[0:3,3].reshape(3,1))

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
    start = time.time()
    F,ck, idx = ICP_.compute_icp_transform(dk, search_method="BruteForce")
    end = time.time()
    BruteForce_time = end - start

    start = time.time()
    F,ck, idx = ICP_.compute_icp_transform(dk, search_method="Octree")
    end = time.time()
    Octree_time = end - start
    log.info(f"ICP time using BruteForce= \n{BruteForce_time} seconds")
    log.info(f"ICP time using Octree= \n{Octree_time} seconds")
    log.info(f"ICP transformation matrix :\n R = {F.R} \n t = {F.p}")


    ############################################################################
    ############### Visualization for dk and the surface mesh ##################
    ############################################################################
    fig = plt.figure()
    plt.set_loglevel('warning')
    ax = fig.add_subplot(121, projection='3d')
    ax.scatter(dk[:,0],dk[:,1],dk[:,2], c = 'r', marker = 'x')
    ax.scatter(vertex[:,0], vertex[:,1], vertex[:,2], c = 'b', alpha=0.2, marker='.')
    ax.set_title("dk and the surface mesh")
    sk = np.zeros_like(dk)
    for i in range(Nframes):
        temp = np.reshape(dk[i,:],(3,1))
        sk[i,:] = np.reshape(F @ temp, (3,)) 
    
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(sk[:,0],sk[:,1],sk[:,2], c = 'r', marker = 'x')
    ax2.scatter(vertex[:,0], vertex[:,1], vertex[:,2], c = 'b', alpha=0.2, marker='.')
    ax2.set_title("Freg*dk and the surface mesh")
    plt.show()
    # log.info(f"dk for {name} data: {d_tip}")


if __name__=="__main__":
    main()
    # a = []
    # a.append(np.array([[1,2,3]]))
    # a.append(np.array([[4,5,6]]))
    # a.append(np.array([[7,8,9]]))
    # print(a[-1]-a[-2])
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
    