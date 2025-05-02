import numpy as np
from scipy.spatial.transform import Rotation as R

def load_imu_data(accel_file, gyro_file, timestamps_file):
    accel = np.load(accel_file)
    gyro = np.load(gyro_file)
    full_data = np.loadtxt(timestamps_file)
    print("timestamps: ", full_data.shape)
    print("accel: ", accel.shape)
    return accel, gyro, full_data[:, 0]

def integrate_imu(accel, gyro, timestamps, rate=100):
    dt = 1.0 / rate
    num_steps = accel.shape[0]

    position = np.zeros(3)
    velocity = np.zeros(3)
    orientation = R.from_quat([0, 0, 0, 1])

    gravity = np.array([0, 0, 9.81])

    poses = []
    for i in range(num_steps):
        # Get measurements
        acc = accel[i]
        gyr = gyro[i]

        # Integrate angular velocity
        delta_angle = gyr * dt
        delta_rot = R.from_rotvec(delta_angle)
        # print(delta_rot)
        orientation = orientation * delta_rot

        # Acceleration from body to world frame
        acc_world = orientation.apply(acc) - gravity
        # acc_world = acc + gravity

        # Integrate velocity and position
        velocity += acc_world * dt
        position += velocity * dt

        if i % 10 == 0 and i < len(timestamps):
            t = timestamps[i % 10]
            quat = orientation.as_quat()
            pose_line = f"{t:.6f} {position[0]:.6f} {position[1]:.6f} {position[2]:.6f} {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}"
            poses.append(pose_line)

    return poses

def save_poses_tum_format(poses, output_file):
    with open(output_file, 'w') as f:
        for pose in poses:
            f.write(pose + '\n')

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default="")
    args = parser.parse_args()    
    input = args.input
    accel_file = input+'/acc.npy'
    gyro_file = input+'/gyro.npy'
    timestamps_file = input+'/TartanEVO.tum'
    output_file = 'tum_poses.txt'

    accel, gyro, timestamps = load_imu_data(accel_file, gyro_file, timestamps_file)
    poses = integrate_imu(accel, gyro, timestamps)
    save_poses_tum_format(poses, output_file)
    print(f"Saved {len(poses)} poses to {output_file}")
