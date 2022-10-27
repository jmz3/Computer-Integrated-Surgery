****************************************** UTILITY FILES ***********************************************
PROGRAMS/cispa/CarteFrame.py                        #Cartesian math package
PROGRAMS/cispa/Registration.py                      #3D-3D point cloud registration package
PROGRAMS/cispa/PivotCalibration.py                  #"pivot" calibration package 
PROGRAMS/cispa/DataProcess.py                       #Load and save the data 
PROGRAMS/cispa/ComputeExpectValue.py                #Compute {C_expected}  and read the sensor data {C} 
PROGRAMS/cispa/CorrectDistortion.py                 #Distortion calibration package
PROGRAMS/OUTPUT/                                    #Folder that contains our result

******************************************* EXECUTABLES *************************************************
PROGRAMS/PA2/pa2_problem1_test.py                   #Compute the c_expected value for all frames as required in PA2 Prob1 
PROGRAMS/PA2/pa2_problem2_test.py                   #Test the distortion correction function with own generated data
PROGRAMS/PA2/pa2_problem3_test.py                   #Perform Pivot calibration for corrected data as required in PA2 Prob3
PROGRAMS/PA2/pa2_problem4_test.py                   #Locate the fiducials as required in Prob4
PROGRAMS/PA2/pa2_problem5_test.py                   #Compute the transformation between CT and EM frame F_reg
PROGRAMS/PA2/pa2_problem6_test.py                   #Include all the steps above and compute the tip position w.r.t. CT frame
PROGRAMS/PA2/Data                                   #Data 

***************************************USING INSTRUCTION******************************************
REQUIREMENTS:

This code requires Python 3.9, NumPy and scipy,
To install NumPy, run: pip install numpy 
To install scipy, run: pip install scipy

How to run our code:
First you need to enter the target directory. For example, in my case: 

The Options for every test executables are loaded by:
    --data-dir, -d, default="PA2/Data", help="Input data directory"
    --output-dir, -o, default="PA2/output", help="Output directory"
    --name, -n, default="pa2-debug-a", help="Name of the output file"

For running the whole pipeline (calibration and navigation), run pa2_problem6_test with options:
For running the -debug-e- case：
>>> python3 pa2_problem6_test.py -n pa2-debug-e

For running the -unknown-h- case：
>>> python3  pa2_problem6_test.py -n pa2-unknown-h

For saving the output1 data
>>> python3 pa2_problem3_test.py -n pa2-debug-a
