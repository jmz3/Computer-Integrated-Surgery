import numpy as np
import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from cispa import CorrectDistortion

def main():
    ground_truth = 50 + (200 - 50)*np.random.rand(3,1000)
    distorted_sensor_err = [0.0000000002*ground_truth[0:1,:]**2*ground_truth[1:2,:]**2*ground_truth[2:3,:],
                            0.0000000004*ground_truth[0:1,:]**2*ground_truth[1:2,:]**2*ground_truth[2:3,:],
                            0.0000000001*ground_truth[0:1,:]*ground_truth[2:3,:]**2*ground_truth[2:3,:]**2]
    distorted_sensor_err = 0.2*np.concatenate(distorted_sensor_err, axis=0)
    distorted_sensor_data = ground_truth + distorted_sensor_err

    correction_coeff = CorrectDistortion.fit(distorted_sensor_data, ground_truth)
    corrected_sensor_data = CorrectDistortion.predict(distorted_sensor_data, correction_coeff)

    print(f"for a nonlinear distortion with mean{np.mean(distorted_sensor_err, axis=1)} and std{np.std(distorted_sensor_err, axis=1)}")

    print(f"distortion correction error: {np.mean(np.linalg.norm(ground_truth - corrected_sensor_data.T, axis=0))}")

    return 0

if __name__=="__main__":
    main()