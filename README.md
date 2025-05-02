The final script used to run the experiment is pose_factor_graph.py. 

Our previous attempt to using iSAM2 is found in iSAM2.py. plot_poses.py was used to align and plot trajectories as .jpg files. 
The other scripts (spline_interp.py, imu_integrate.py, test_plot.py) were used to determine the coordinates of the IMU measurements, TartanEVO output, and the accuracy of the IMU measurements. 
csv_concat.py was used to concat the outputs from EVO tool, used to analyze the TartanEVO (with and without GTSAM) trajectories compared to the ground truth trajectories.
