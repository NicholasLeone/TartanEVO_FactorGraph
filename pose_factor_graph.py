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

def read_and_extract_arrays(txt_file, file, transform = R.from_matrix(np.eye(3))):
    input = os.path.join(txt_file, file+".tum")
    full_data = np.loadtxt(input)

    # Extract only the x, y, z columns (column indices 1, 2, and 3)
    timestamps = full_data[:, 0]
    pose_data = full_data[:, 1:]
    # Extract quaternion part: columns 3 to 6 (qx, qy, qz, qw)
    quaternions = pose_data[:, 3:7]

    # Convert to Euler angles (in radians)
    rot = R.from_quat(quaternions)  # expects [x, y, z, w]
    transformed_rot = transform * rot
    eulers = transformed_rot.as_euler('xyz', degrees=False)  # returns (roll, pitch, yaw)

    # Combine with position if needed
    positions = transform.apply(pose_data[:, :3])
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

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default="")
    args = parser.parse_args()
    input = args.input  

    # Declare the 3D translational standard deviations of the prior factor's Gaussian model, in meters.
    prior_xyz_sigma = 1e-10

    # Declare the 3D rotational standard deviation of the prior factor's Gaussian model, in radians.
    prior_theta_sigma = 1e-10

    # Declare the 3D translational standard deviations of the odometry factor's Gaussian model, in meters.
    odometry_xyz_sigma = 1e-2

    # Declare the 3D rotational standard deviation of the odometry factor's Gaussian model, in radians.
    odometry_theta_sigma = 5e-2

    soft_prior_xyz_sigma = 1e-2
    soft_prior_theta_sigma = 1e-2    
    
    # accelerometer bias random work noise standard deviation.  #0.02
    acc_w = 1e-5
    gyr_w = 1e-5  

    # Create Preint IMU Parameter object
    # gravity = np.array([[0.0], [0.0], [9.81]]).astype(np.float64)
    # pim_params = gtsam.PreintegrationParams(gravity)
    pim_params = gtsam.PreintegrationParams.MakeSharedD(9.81)
    accel_sigma = 1e-3
    gyro_sigma = 1e-4
    I_3x3 = np.eye(3)
    pim_params.setGyroscopeCovariance(gyro_sigma ** 2 * I_3x3)
    pim_params.setAccelerometerCovariance(accel_sigma ** 2 * I_3x3)
    pim_params.setIntegrationCovariance(1e-3 * I_3x3)

    # Assume zero bias
    prev_bias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))

    # Set PRIOR_POSE_NOISE, ODOMETRY_NOISE, and SOFT_PRIOR_NOISE
    PRIOR_POSE_NOISE = noise_model(prior_theta_sigma, prior_xyz_sigma)

    ODOMETRY_NOISE = noise_model(odometry_theta_sigma, odometry_xyz_sigma)  

    # Set PRIOR_VELOCITY_NOISE
    PRIOR_VELOCITY_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 1e-10)
    VELOCITY_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 1e-2)

    # Set BIAS_NOISE and PRIOR_BIAS_NOISE
    biasN = np.array([acc_w, acc_w, acc_w, gyr_w, gyr_w, gyr_w])
    BIAS_NOISE = noise_model(acc_w, gyr_w)
    PRIOR_BIAS_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 1e-10)

    # Define the PreintegratedImuMeasurements object here.
    pim = gtsam.PreintegratedImuMeasurements(pim_params, prev_bias)

    # Create a Nonlinear factor graph as well as the data structure to hold state estimates.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    axis_sequence = "XYZ"

    # Get absolute and relative poses, and timestamps from original VO output
    poses_init, delta_init, timestamps = read_and_extract_arrays(input, 'TartanEVO')
    poses_trans = poses_init[:, :3]
    poses_rot = poses_init[:, 3:]
    poses = np.hstack([poses_trans, poses_rot])
    
    delta_trans = delta_init[:, :3]
    delta_rot = delta_init[:, 3:]
    delta = np.hstack([delta_trans, delta_rot])
    
    poses_gt, _, _ = read_and_extract_arrays(input, 'ground_truth')
    
    acc = npy_load(input, "acc")
    # acc = convert_imu_to_vo(acc_imu)

    gyro = npy_load(input, "gyro") 
    # gyro = convert_imu_to_vo(gyro_imu)

    vel = npy_load(input, "vel_global")

    # Set initial pose
    x0, y0, z0, roll0, pitch0, yaw0 = poses[0, :]
    rot0 = R.from_euler(axis_sequence, [roll0, pitch0, yaw0], degrees=False)
    # prev_pose = gtsam.Pose3(r = gtsam.Rot3(rot0.as_matrix().astype(np.float64)), t = gtsam.Point3(x0, y0, z0))
    # prev_pose = gtsam.Pose3(r = gtsam.Rot3(np.eye(3).astype(np.float64)), t = gtsam.Point3(0, 0, 0))
    prev_pose = gtsam.Pose3(r = gtsam.Rot3(R.from_euler(axis_sequence, poses_gt[0, 3:], degrees=False).as_matrix()), t = gtsam.Point3(poses_gt[0, :3]))

    # Set initial velocity
    # prev_vel = gtsam.Point3(0.0, 0.0, 0.0)
    prev_vel = gtsam.Point3(vel[0])
    prev_state = gtsam.NavState(prev_pose, prev_vel)

    # Push priors
    graph.push_back(gtsam.PriorFactorPose3(X(0), prev_pose, PRIOR_POSE_NOISE))
    graph.push_back(gtsam.PriorFactorVector(V(0), prev_vel, PRIOR_VELOCITY_NOISE))
    graph.push_back(gtsam.PriorFactorConstantBias(B(0), prev_bias, PRIOR_BIAS_NOISE))
    initial_estimate.insert(X(0), prev_pose)
    initial_estimate.insert(B(0), prev_bias)
    initial_estimate.insert(V(0), prev_vel)

    imu_count = 0
    axis_sequence = "XYZ"

    # print("First acc: ", acc[:50])
    # print("First five poses: ", poses[:5])

    odom_noise = ODOMETRY_NOISE

    for i in range(1, len(poses)):
        # Get TartanEVO output
        dx, dy, dz, droll, dpitch, dyaw = delta[i - 1]
        drot = R.from_euler(axis_sequence, [droll, dpitch, dyaw], degrees=False)
        delta_pose = gtsam.Pose3(r=gtsam.Rot3(drot.as_matrix()), t=gtsam.Point3(dx, dy, dz))

        # Preintegrate IMU measurements
        for j in range(10):
            if imu_count < len(acc):
                pim.integrateMeasurement(acc[imu_count], gyro[imu_count], 0.01)

        # Push BetweenFactors
        graph.push_back(gtsam.ImuFactor(X(i - 1), V(i - 1), X(i), V(i), B(i), pim))
        graph.push_back(gtsam.BetweenFactorPose3(X(i - 1), X(i), delta_pose, odom_noise))
        graph.push_back(gtsam.BetweenFactorConstantBias(B(i - 1), B(i), gtsam.imuBias.ConstantBias(), BIAS_NOISE))

        # Estiamte and push estimates into factor graph
        navState = pim.predict(prev_state, prev_bias)
        computed_pose_estimate = prev_pose.compose(delta_pose)
        initial_estimate.insert(V(i), navState.velocity())
        # initial_estimate.insert(X(i), computed_pose_estimate)
        initial_estimate.insert(X(i), navState.pose())
        initial_estimate.insert(B(i), prev_bias)

        # prev_pose = computed_pose_estimate
        # Set prev values to estimated values
        prev_pose = navState.pose()
        prev_vel = navState.velocity()
        prev_state = navState
        pim.resetIntegration()

    # Perform full batch optimization
    print("Starting final batch optimization...")
    lm_params = gtsam.LevenbergMarquardtParams()
    lm_params.setVerbosityLM("SUMMARY")
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, lm_params)
    result = optimizer.optimize()
    # result = initial_estimate
    print("Optimization complete.")
    
    # Record final trajectory
    final_traj = np.zeros((len(poses), 7))
    # test_traj = []
    for i in range(len(poses)):
        curr_pose = result.atPose3(X(i))
        # test_traj.append(curr_pose.translation())
        final_traj[i, :3] = curr_pose.translation()
        final_traj[i, 3:] = curr_pose.rotation().quaternion()

    write_traj(input, final_traj, timestamps, "CountryHouse-P000-gtsam_batch.tum")