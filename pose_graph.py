import gtsam.imuBias
import gtsam.noiseModel
import matplotlib.pyplot as plt
import numpy as np
import os
import math
import gtsam
import gtsam.utils.plot as gtsam_plot
from scipy.spatial.transform import Rotation as R
from gtsam.symbol_shorthand import B, V, X

# Make sure angle is within -pi to pi
def normalize_angle(angle_diff):
    return (angle_diff + np.pi) % (2 * np.pi) - np.pi

# Load .npy files
def npy_load(dir, file):
    input = os.path.join(dir, file + ".npy")
    full_data = np.load(input)
    return full_data    

def read_and_extract_arrays(txt_file):
    input = os.path.join(txt_file, "TartanEVO.tum")
    full_data = np.loadtxt(input)

    # Extract only the x, y, z columns (column indices 1, 2, and 3)
    timestamps = full_data[:, 0]
    pose_data = full_data[:, 1:]
    # Extract quaternion part: columns 3 to 6 (qx, qy, qz, qw)
    quaternions = pose_data[:, 3:7]

    # Convert to Euler angles (in radians)
    rot = R.from_quat(quaternions)  # expects [x, y, z, w]
    eulers = rot.as_euler('xyz', degrees=False)  # returns (roll, pitch, yaw)

    # Combine with position if needed
    positions = pose_data[:, :3]
    full_output = np.hstack([positions, eulers])

    # Compute relative poses by subtracting absolute poses between each timestep
    delta_output = full_output[1:] - full_output[:-1]
    delta_output[:, 3:] = normalize_angle(delta_output[:, 3:])    

    return full_output, delta_output, timestamps

# Save trajectory in tum format
def write_traj(output_dir, poses, timestamps, filename="tartanevo_optimize.tum"): 
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, filename)

    with open(output_path, 'w') as f:
        for i in range(len(poses) - 1):
            # GTSAM saves quaternion in qw, qx, qy, qz
            x, y, z, qw, qx, qy, qz = poses[i]

            #TUM formats quaternions as qx, qy, qz, qw
            line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
                timestamps[i], x, y, z, qx, qy, qz, qw
            )
            f.write(line)

def noise_model(theta_sigma, xyz_sigma):
    return gtsam.noiseModel.Diagonal.Sigmas(np.array([theta_sigma, theta_sigma, theta_sigma, xyz_sigma, xyz_sigma, xyz_sigma]))



# Example: python pose_graph.py --input tartanevo_gtsam/tartanairv2/CountryHouse/P000/
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default="")
    args = parser.parse_args()
    input = args.input

    # Create isam object
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    parameters.setRelinearizeSkip(1)
    isam = gtsam.ISAM2(parameters)    

    # Declare the 3D translational standard deviations of the prior factor's Gaussian model, in meters.
    prior_xyz_sigma = 1e-1

    # Declare the 3D rotational standard deviation of the prior factor's Gaussian model, in radians.
    prior_theta_sigma = 5e-1

    # Declare the 3D translational standard deviations of the odometry factor's Gaussian model, in meters.
    odometry_xyz_sigma = 5e-2

    # Declare the 3D rotational standard deviation of the odometry factor's Gaussian model, in radians.
    odometry_theta_sigma = 1e-1

    soft_prior_xyz_sigma = 1.25e-1
    soft_prior_theta_sigma = 1.25e-1


    # Set PRIOR_POSE_NOISE, ODOMETRY_NOISE, and SOFT_PRIOR_NOISE
    PRIOR_POSE_NOISE = noise_model(prior_theta_sigma, prior_xyz_sigma)

    ODOMETRY_NOISE = noise_model(odometry_theta_sigma, odometry_xyz_sigma)
    SOFT_PRIOR_NOISE = noise_model(soft_prior_theta_sigma, soft_prior_xyz_sigma)    

    # Set PRIOR_VELOCITY_NOISE
    PRIOR_VELOCITY_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 1e-3)
    
    # accelerometer bias random work noise standard deviation.  #0.02
    acc_w = 0.02    
    gyr_w = 1e-3
    
    # Set BIAS_NOISE and PRIOR_BIAS_NOISE
    biasN = np.array([acc_w, acc_w, acc_w, gyr_w, gyr_w, gyr_w])
    BIAS_NOISE = noise_model(acc_w, gyr_w)
    PRIOR_BIAS_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)


    # Get absolute and relative poses, and timestamps from original VO output
    poses, delta, timestamps = read_and_extract_arrays(input)
    acc = npy_load(input, "acc")
    gyro = npy_load(input, "gyro")    

    # Create Preint IMU Parameter object
    pim_params = gtsam.PreintegrationParams.MakeSharedD(9.80511)
    gyro_sigma = 0.01
    accel_sigma = 0.02
    I_3x3 = np.eye(3)
    pim_params.setGyroscopeCovariance(gyro_sigma**2 * I_3x3)
    pim_params.setAccelerometerCovariance(accel_sigma**2 * I_3x3)
    pim_params.setIntegrationCovariance(1e-5**2 * I_3x3)

    # Assume zero bias
    prev_bias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))

    # Define the PreintegratedImuMeasurements object here.
    pim = gtsam.PreintegratedImuMeasurements(pim_params, prev_bias)

    # Create a Nonlinear factor graph as well as the data structure to hold state estimates.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    axis_sequence = "XYZ"

    # Set initial pose
    x0, y0, z0, roll0, pitch0, yaw0 = poses[0, :]
    rot0 = R.from_euler(axis_sequence, [roll0, pitch0, yaw0], degrees=False)
    prev_pose = gtsam.Pose3(r = gtsam.Rot3(rot0.as_matrix().astype(np.float64)), t = gtsam.Point3(x0, y0, z0))
    # prev_pose = gtsam.Pose3(r = gtsam.Rot3(np.eye(3).astype(np.float64)), t = gtsam.Point3(0, 0, 0))

    # Set initial velocity
    prev_vel = gtsam.Point3(0.0, 0.0, 0.0)
    prev_state = gtsam.NavState(prev_pose, prev_vel)

    # Push priors
    graph.push_back(gtsam.PriorFactorPose3(X(0), prev_pose, PRIOR_POSE_NOISE))
    graph.push_back(gtsam.PriorFactorVector(V(0), prev_vel, PRIOR_VELOCITY_NOISE))
    graph.push_back(gtsam.PriorFactorConstantBias(B(0), prev_bias, PRIOR_BIAS_NOISE))
    initial_estimate.insert(X(0), prev_pose)
    initial_estimate.insert(B(0), prev_bias)
    initial_estimate.insert(V(0), prev_vel)

    # Update once
    isam.update(graph, initial_estimate)
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate.clear()

    # IMU preintegration counter
    imu_count = 0


    for i in range(1, len(poses)):


        # Obtain the odometry that is received by the VO algorithm.
        dx, dy, dz, droll, dpitch, dyaw = delta[i - 1]
        drot = R.from_euler(axis_sequence, [droll, dpitch, dyaw], degrees=False)
        delta_pose = gtsam.Pose3(r = gtsam.Rot3(drot.as_matrix().astype(np.float64)), t = gtsam.Point3(dx, dy, dz))

        # Obtain Absolute Pose from VO algorithm as soft anchor
        x, y, z, roll, pitch, yaw = poses[i]
        rot = R.from_euler(axis_sequence, [droll, dpitch, dyaw], degrees=False)
        pose = gtsam.Pose3(r = gtsam.Rot3(rot.as_matrix().astype(np.float64)), t = gtsam.Point3(x, y, z))        
        
        # Preintegrate imu measurements
        for j in range(10):
                if imu_count < len(acc):
                    pim.integrateMeasurement(acc[imu_count], gyro[imu_count], 0.01)
                    imu_count += 1
        # Every 100 iterations (0.1*100 = 10 seconds), push anchor as prior
        if i % 100 == 0:
            graph.push_back(gtsam.PriorFactorPose3(X(i), pose, SOFT_PRIOR_NOISE))
                # graph.push_back(gtsam.PriorFactorPose3(X(i), prev_pose, SOFT_PRIOR_NOISE))
        
        # Push IMU factor
        factor = gtsam.ImuFactor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), pim)
        graph.push_back(factor)        
        
        # Push relative pose from VO into graph
        graph.push_back(gtsam.BetweenFactorPose3(X(i - 1), X(i), delta_pose, ODOMETRY_NOISE))

        # Push bias into graph
        # graph.push_back(gtsam.BetweenFactorConstantBias(B(i - 1), B(i), gtsam.imuBias.ConstantBias(), gtsam.noiseModel.Diagonal.Sigmas(np.sqrt(pim.deltaTij())*biasN)))
        graph.push_back(gtsam.BetweenFactorConstantBias(B(i - 1), B(i), gtsam.imuBias.ConstantBias(), PRIOR_BIAS_NOISE))  

        # Estimate current pose and velocity
        computed_pose_estimate = prev_pose.compose(delta_pose)
        navState = pim.predict(prev_state, prev_bias) 
    
        # Insert prediced pose, velocity and bias
        initial_estimate.insert(V(i), navState.velocity())
        initial_estimate.insert(X(i), navState.pose()) # Predicted pose based on IMU
        # initial_estimate.insert(X(i), computed_pose_estimate) # Predicted pose based on relative pose + previous pose
        initial_estimate.insert(B(i), prev_bias)        

        # Perform incremental update to iSAM2's internal Bayes tree, optimizing only the affected variables.
        print("Timestep: ", i*0.1)
        isam.update(graph, initial_estimate)
        isam.update()

        # Set prev variables to estimated values
        result = isam.calculateEstimate()
        prev_pose = result.atPose3(X(i))
        prev_vel = result.atPoint3(V(i))
        prev_bias = result.atConstantBias(B(i))
        prev_state = gtsam.NavState(prev_pose, prev_vel)
        
        # Reset graph, values, and IMU integration object
        graph.resize(0)       
        pim.resetIntegrationAndSetBias(prev_bias)
        initial_estimate.clear()
    
    # Record final trajectory
    final_traj = np.zeros((len(poses), 7))
    for i in range(len(poses)):
        curr_pose = result.atPose3(X(i))
        final_traj[i, :3] = curr_pose.translation()
        final_traj[i, 3:] = curr_pose.rotation().quaternion()
    
    # Print covariance for pose factors
    i = 1
    while result.exists(X(i)):
        if i % 20 == 0:
            print("X{%d} covariance:", i, isam.marginalCovariance(X(i - 1)))
        i += 1
    
    # Save final trajectory
    write_traj(input, final_traj, timestamps, "CountryHouse-P000-gtsam_isam2.tum")
