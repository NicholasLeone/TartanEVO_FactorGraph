#!/bin/bash

# Check if the argument is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <command>"
  echo "Options for <command>: evo_traj, evo_rpe"
  exit 1
fi

# Store the command in a variable
COMMAND=$1

# Define the base directories for the results and plots
DEVO_RESULTS_BASE="GTSAM_Output"
PLOTS_BASE="plots"

ENVS=("tartanevo_gtsam/tartanairv2")

TRAJS=("ShoreCaves/P001" "ShoreCaves/P005" "CountryHouse/P000" "CountryHouse/P001" "CountryHouse/P002" "CountryHouse/P003" "CountryHouse/P004" "CountryHouse/P005"
"MiddleEast/P000" "MiddleEast/P001" "MiddleEast/P002" "MiddleEast/P003" "CoalMine/P000")
# Loop over each environment and process the corresponding trajectories
for ENV in "${ENVS[@]}"; do
  for TRAJ in "${TRAJS[@]}"; do
    echo "Processing environment: $ENV"

    # Define the file paths
    # GROUND_TRUTH_TUM="$ENV/pose_lcam_front_tum.txt"
    GROUND_TRUTH_TUM="$ENV/$TRAJ/ground_truth.tum"
    GTSAM_TUM="$ENV/$TRAJ/CountryHouse-P000-gtsam_batch.tum"
    TEVO_TUM="$ENV/$TRAJ/TartanEVO.tum"
    ZIP_DIR="$PLOTS_BASE/$TRAJ/"
    # PLOT_DIR="$PLOTS_BASE/$ENV"

    mkdir -p $PLOTS_BASE
    mkdir -p $ZIP_DIR
    mkdir -p $ZIP_DIR/results
    mkdir -p $ZIP_DIR/evo_traj
    

    # Run the evo_traj command
    if [ "$COMMAND" == "evo_traj" ]; then
      evo_traj tum "$GTSAM_TUM" --ref="$GROUND_TRUTH_TUM" --save_plot="$PLOT_BASE"/evo_traj/"$TRAJ" --plot_mode=xy --t_max_diff=0.1 --align --correct_scale
    elif [ "$COMMAND" == "evo_rpe" ]; then
      evo_rpe tum "$GROUND_TRUTH_TUM" "$GTSAM_TUM" --save_plot="$ZIP_DIR"GTSAM_rpe_gtsam --align --correct_scale --t_max_diff=0.005 --delta=10.0 --delta_unit=m --pose_relation point_distance_error_ratio --all_pairs --save_results "$ZIP_DIR"GTSAM_rpe_gtsam.zip
      evo_rpe tum "$GROUND_TRUTH_TUM" "$TEVO_TUM" --save_plot="$ZIP_DIR"TartanEVO_rpe_gtsam --align --correct_scale --t_max_diff=0.005 --delta=10.0 --delta_unit=m --pose_relation point_distance_error_ratio --all_pairs --save_results "$ZIP_DIR"TartanEVO_rpe_gtsam.zip
    elif [ "$COMMAND" == "evo_ape" ]; then
      evo_ape tum "$GROUND_TRUTH_TUM" "$GTSAM_TUM" --save_plot="$ZIP_DIR"GTSAM_ape_gtsam --align --correct_scale --t_max_diff=0.005 --save_results "$ZIP_DIR"GTSAM_ape_gtsam.zip
      evo_ape tum "$GROUND_TRUTH_TUM" "$TEVO_TUM" --save_plot="$ZIP_DIR"TartanEVO_ape_gtsam --align --correct_scale --t_max_diff=0.005 --save_results "$ZIP_DIR"TartanEVO_ape_gtsam.zip
    else
      echo "Invalid command: $COMMAND"
      exit 1
    fi
    
    if [ "$COMMAND" == "evo_rpe" ]; then
      evo_res "$ZIP_DIR"TartanEVO_rpe_gtsam.zip --save_table "$ZIP_DIR"results/TartanEVO_rpe.csv
      evo_res "$ZIP_DIR"GTSAM_rpe_gtsam.zip --save_table "$ZIP_DIR"results/GTSAM_rpe.csv
    elif [ "$COMMAND" == "evo_ape" ]; then
      evo_res "$ZIP_DIR"TartanEVO_ape_gtsam.zip --save_table "$ZIP_DIR"results/TartanEVO_ape.csv
      evo_res "$ZIP_DIR"GTSAM_ape_gtsam.zip --save_table "$ZIP_DIR"results/GTSAM_ape.csv
    fi

    echo "Finished processing environment: $ENV"
  done
  #else
  #    echo "Invalid command: $COMMAND"
  #    exit 1

done
