import matplotlib.pyplot as plt
import numpy as np
import os
from evo.tools import file_interface
from evo.core import sync

def read_and_extract_arrays(txt_file):
    # Load the entire file into a NumPy array
    full_data = np.loadtxt(txt_file)

    # Extract only the x, y, z columns (column indices 1, 2, and 3)
    xyz_data = full_data[:, 1:4]

    return full_data, xyz_data


def plot_trajectory_poses(gt_trajectory, trajectory_1, trajectory_2, save_dir, file_name):
    """
    Plot 2D trajectory poses one at a time and save each plot in the specified directory.

    :param trajectory: (N, 2) numpy array of 2D trajectory points (x, y)
    :param save_dir: directory where figures will be saved
    """
    # Ensure the save directory exists
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Initialize plot
    plt.figure(figsize=(6, 6))
    plt.xlim(-10, 10)  # Adjust according to your data
    plt.ylim(-10, 10)  # Adjust according to your data
    plt.gca().set_aspect('equal', adjustable='box')  # Keep the aspect ratio equal    
    plt.clf()

    # Plot trajectories
    plt.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], linestyle='--', color='k', label='ground_truth')
    plt.plot(trajectory_1[:, 0], trajectory_1[:, 1], color='b', label='TartanEVO + GTSAM')
    plt.plot(trajectory_2[:, 0], trajectory_2[:, 1], color='g', label='TartanEVO')

    # Add labels and title
    plt.title(f"{file_name}")
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.legend(loc='upper right')
    plt.savefig(os.path.join(save_dir, f"{file_name}.png"))

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default="")
    parser.add_argument('--devo', default="")
    parser.add_argument('--tartanevo', default="")
    parser.add_argument('--ref', default="")
    args = parser.parse_args()
    devo = args.devo
    tartanevo = args.tartanevo
    ref = args.ref
    input = args.input
    input_split = input.split('/')
    traj_name = input_split[-3] + '_' + input_split[-2]

    if input != "":
        traj_ref = file_interface.read_tum_trajectory_file(input + "ground_truth.tum")
        traj_devo = file_interface.read_tum_trajectory_file(input + "CountryHouse-P000-gtsam_batch.tum")
        # traj_devo = file_interface.read_tum_trajectory_file(input + "tum_poses.txt")
        traj_tevo = file_interface.read_tum_trajectory_file(input + "TartanEVO.tum")
    else:
        traj_ref = file_interface.read_tum_trajectory_file(ref)
        traj_devo = file_interface.read_tum_trajectory_file(devo)
        traj_tevo = file_interface.read_tum_trajectory_file(tartanevo)

    # devo_full, devo_xyz = read_and_extract_arrays(devo)
    # ref_full, ref_xyz = read_and_extract_arrays(ref)
    # tartanevo_full, tartanevo_xyz = read_and_extract_arrays(tartanevo)
    

    traj_ref_devo_sync, traj_devo = sync.associate_trajectories(traj_ref, traj_devo)
    traj_ref_tevo_sync, traj_tevo = sync.associate_trajectories(traj_ref, traj_tevo)

    traj_devo.align(traj_ref_devo_sync, correct_scale=True, correct_only_scale=False)
    traj_tevo.align(traj_ref_tevo_sync, correct_scale=True, correct_only_scale=False)

    # traj_devo.align_origin(traj_ref=traj_ref_devo_sync)
    # traj_tevo.align_origin(traj_ref=traj_ref_tevo_sync)

    plot_trajectory_poses(gt_trajectory=traj_ref.positions_xyz, trajectory_1=traj_devo.positions_xyz, trajectory_2=traj_tevo.positions_xyz, save_dir='matplot_fig', file_name = traj_name)
