import gtsam
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
from gtsam.symbol_shorthand import X, V, B

def read_and_extract_arrays(txt_file, path):
    input = os.path.join(txt_file, path + ".tum")
    full_data = np.loadtxt(input)

    pose_data = full_data[:, 1:]
    quaternions = pose_data[:, 3:7]
    rot = R.from_quat(quaternions)
    eulers = rot.as_euler('xyz', degrees=False)

    positions = pose_data[:, :3]
    full_output = np.hstack([positions, eulers])   

    return full_output

def load_npy(path, name):
    return np.load(os.path.join(path, name + '.npy'))

def load_gt_poses(path):
    poses = np.load(os.path.join(path, 'poses_gt.npy'))
    return poses

def integrate_imu(imu_acc, imu_gyro, dt, gt_pose0, gt_vel0):
    n = imu_acc.shape[0]
    pim_params = gtsam.PreintegrationParams.MakeSharedD(9.81)
    pim_params.setAccelerometerCovariance(np.eye(3)*1e-2)
    pim_params.setGyroscopeCovariance(np.eye(3)*1e-2)
    pim_params.setIntegrationCovariance(np.eye(3)*1e-2)

    bias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))

    rot0 = R.from_euler('xyz', gt_pose0[3:]).as_matrix()
    pose0 = gtsam.Pose3(gtsam.Rot3(rot0), gtsam.Point3(*gt_pose0[:3]))
    # pose0 = gtsam.Pose3(gtsam.Rot3(np.eye(3)), gtsam.Point3(np.zeros(3)))
    vel0 = gtsam.Point3(*gt_vel0)
    state = gtsam.NavState(pose0, vel0)

    # Preintegrator
    pim = gtsam.PreintegratedImuMeasurements(pim_params, bias)

    poses_pred = [pose0]
    velocities = [vel0]
    imu_count = 0
    for i in range(1, n // 10):
        for j in range(10):
            pim.integrateMeasurement(imu_acc[imu_count], imu_gyro[imu_count], dt)
            imu_count += 1
        state = pim.predict(state, bias)
        poses_pred.append(state.pose())
        velocities.append(state.velocity())
        pim.resetIntegration()

    return poses_pred, velocities

def pose3_to_xyz_rpy(pose3):
    t = pose3.translation()
    Rmat = pose3.rotation().matrix()
    euler = R.from_matrix(Rmat).as_euler('xyz')
    return np.array([t[0], t[1], t[2], *euler])

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=str, required=True, help='Directory containing acc.npy, gyro.npy, poses_gt.npy, timestamps.npy')
    args = parser.parse_args()

    acc = load_npy(args.input, 'acc')
    vel = load_npy(args.input, 'vel_global')
    gyro = load_npy(args.input, 'gyro')
    poses_gt = read_and_extract_arrays(args.input, 'ground_truth')
    poses_tevo = read_and_extract_arrays(args.input, 'TartanEVO')
    timestamps = load_npy(args.input, 'imu_time')

    dt_array = np.diff(timestamps)
    dt = 0.01
    gt_pose0 = poses_gt[0]
    gt_vel0 = vel[0]

    predicted_poses, predicted_vels = integrate_imu(acc, gyro, dt, gt_pose0, gt_vel0)
    predicted_array = np.array([pose3_to_xyz_rpy(pose) for pose in predicted_poses])

    np.save(os.path.join(args.input, 'imu_predicted_poses.npy'), predicted_array)

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(poses_gt[:, 0], poses_gt[:, 1], label='GT', linewidth=2)
    plt.plot(predicted_array[:, 0], predicted_array[:, 1], '--', label='IMU Integrated', linewidth=2)
    plt.plot(-poses_tevo[:, 1], poses_tevo[:, 0], '--', label='TartanEVO', linewidth=2)
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.legend()
    plt.axis('equal')
    plt.title('IMU Trajectory vs Ground Truth')
    plt.grid()
    plt.show()
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')

    # ax.plot3D(predicted_array[:, 0], predicted_array[:, 1], predicted_array[:, 2], label='IMU integrated', color='blue')
    # ax.plot3D(poses_gt[:, 0], poses_gt[:, 1], poses_gt[:, 2], label='IMU integrated', color='blue')\

    # ax.plot3D(-poses_tevo[:, 1], poses_tevo[:, 0], poses_tevo[:, 2], color='red', label='VO poses')

    # ax.set_title("Trajectory Comparison")
    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    # ax.legend()
    # ax.grid(True)

    # plt.tight_layout()
    # plt.show()
