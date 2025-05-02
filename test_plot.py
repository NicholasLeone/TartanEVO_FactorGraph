import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from scipy.spatial.transform import Rotation as R
from itertools import permutations

parser = argparse.ArgumentParser()
parser.add_argument('--input', default="")
args = parser.parse_args()
input = args.input

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

def convert_imu_to_vo(acc_or_gyro):
    # acc_or_gyro: np.ndarray of shape (N, 3)
    return np.stack([-acc_or_gyro[:, 1],  # East → Right (X')
                     acc_or_gyro[:, 0],  # Down → Down (Y')
                     acc_or_gyro[:, 2]], # North → Forward (Z')
                    axis=1)


# ==== Replace with your actual data ====
# First 50 accelerometer values (100 Hz)
poses_init, delta, timestamps = read_and_extract_arrays(input)
poses_classic = npy_load(input, "pos_global")
velocity = npy_load(input, "vel_global")
acc = npy_load(input, "acc_nograv")
ori = npy_load(input, "ori_global")
gyro = npy_load(input, "gyro")
# acc[:, -1] -= 9.81
print("velocity init: ", velocity[0])
g = np.array([0, 0, -9.81])
# ========== IMU integration ==========

dt = 0.01  # 100 Hz
trans = convert_imu_to_vo(poses_init[:, :3])
rot = convert_imu_to_vo(poses_init[:, 3:])
poses = np.hstack([trans, rot])
pos = poses[0, :3].copy()
vel = np.zeros(3)
positions = [pos.copy()]
orientation = R.from_rotvec(poses[0, 3:])
# poses = poses[:20]
# acc = acc[:200]

print(poses[:10])
print(poses_classic[:10])

for i in range(len(acc)):
    omega = gyro[i]
    delta_angle = omega * dt
    delta_rot = R.from_rotvec(delta_angle)
    orientation = orientation * delta_rot
    # orientation = R.from_rotvec(ori[i])

    # Rotate acc to world frame
    acc_world = acc[i]

    # Integrate
    # pos += vel * dt + 0.5 * acc_world * dt ** 2
    # vel = acc_world * dt
    pos += orientation.apply(velocity[i] * dt)
    # pos += vel * dt + 0.5 * acc_world * dt ** 2
    positions.append(pos.copy())

imu_positions = np.array(positions)

# poses_classic_trans = poses_classic[::10, :3]
# poses_trans = poses[:, :3]
# min_error = np.inf

for perm in permutations([0, 1, 2]):
    print("perm: ", perm)\

# print("Best permutation of VO columns:", best_perm)
# print("Minimum MSE:", min_error)


# ========== Plotting ==========

fig = plt.figure()
ax = plt.axes(projection='3d')

# IMU trajectory
# ax.plot3D(imu_positions[:, 0], imu_positions[:, 1], imu_positions[:, 2], label='IMU integrated', color='blue')
ax.plot3D(poses_classic[:, 0], poses_classic[:, 1], poses_classic[:, 2], label='IMU integrated', color='blue')

# VO poses
ax.plot3D(poses[:, 0], poses[:, 1], poses[:, 2], color='red', label='VO poses')

ax.set_title("Trajectory Comparison")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.show()
