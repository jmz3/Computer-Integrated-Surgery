****************************************** UTILITY FILES ***********************************************
PROGRAMS/cispa/FindBoundingSphere.py                #Generate a bounding sphere for a given triangle
PROGRAMS/cispa/FindClosestPoint2Triangle.py         #Find the closest point from a point to a triangle
PROGRAMS/cispa/FindClosestPoint2Mesh.py             #Package that contains several methods to find closest point on mesh
PROGRAMS/cispa/Octree.py	                    #Generate an Octree for given triangular mesh
PROGRAMS/OUTPUT/                                    #Folder that contains our result

******************************************* EXECUTABLES *************************************************
PROGRAMS/PA3/pa3_main.py                  	    #main file that run both search methods and print the time consumption
PROGRAMS/PA3/pa3_compute_dk_test.py                 #Test the compute dk algorithm
PROGRAMS/PA3/pa3_find_closest_on_triangle.py        #Test the closest point on triangle
PROGRAMS/PA3/pa3_octree_search_test.py              #Pure Octree Search test
PROGRAMS/PA3/pa3_linear_search_test.py              #Pure Brute Force Search test
PROGRAMS/PA3/Data                                   #Data 

***************************************** USING INSTRUCTION **********************************************
REQUIREMENTS:

This code requires Python 3.9, NumPy and scipy, matplotlib
To install NumPy, run: pip install numpy 
To install scipy, run: pip install scipy
To install matplotlib, run: pip install matplotlib

How to run our code:
First you need to enter the target directory, i.e.
$ cd ~/*PARENT DIR*/PROGRAMS

The Options for every test executables are loaded by:
    --data-dir, -d,     default="PA3/Data"      Specify the input data directory
    --output-dir, -o,   default="PA3/output"    Specify the output directory"
    --name, -n,         default="PA3-A-Debug"   Specify the Name of the output file

******************************************* Examples ***************************************************
For running the whole pipeline (calibration and navigation), run pa3_main.py with options:
For running the -debug-e- case：
>>> python3 pa3_main.py -n PA3-B-Debug
Note: Running pa3_main.py will automatically save the required result to "*-output.txt"

For running the -unknown-h- case：
>>> python3  pa3_main.py -n PA3-A-Unknown

For visualizing the code result
>>> python3 pa3_linear_search_test.py
Or
>>> python3 pa3_find_closest_on_triangle.py 
Or
>>> python3 pa3_octree_search_test.py