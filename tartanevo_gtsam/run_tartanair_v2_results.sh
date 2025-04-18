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

#ENVS=("CountryHouseAutoExposure_Data_easy_P000" "CountryHouseAutoExposure_Data_easy_P001" "CountryHouseAutoExposure_Data_easy_P002" "CountryHouseAutoExposure_Data_easy_P003" "CountryHouseAutoExposure_Data_easy_P004" "CountryHouseAutoExposure_Data_easy_P005" "MiddleEastAutoExposure_Data_easy_P000" "MiddleEastAutoExposure_Data_easy_P001" "MiddleEastAutoExposure_Data_easy_P002" "MiddleEastAutoExposure_Data_easy_P003" "ShoreCavesAutoExposure_Data_easy_P001" "ShoreCavesAutoExposure_Data_easy_P002" "ShoreCavesAutoExposure_Data_easy_P003" "ShoreCavesAutoExposure_Data_easy_P004" "ShoreCavesAutoExposure_Data_easy_P005" "ShoreCavesAutoExposure_Data_easy_P006" "ShoreCavesAutoExposure_Data_easy_P007" "ShoreCavesAutoExposure_Data_easy_P008")

# ENVS=("CountryHouse/P000" "CountryHouse/P001" "CountryHouse/P002" "CountryHouse/P003" "CountryHouse/P004" "CountryHouse/P005")
# ENVS=("MiddleEast/P000" "MiddleEast/P001" "MiddleEast/P002" "MiddleEast/P003")
# ENVS=("ShoreCaves/P001" "ShoreCaves/P005")
# ENVS=("ShoreCaves/P002" "ShoreCaves/P003" "ShoreCaves/P004" "ShoreCaves/P006" "ShoreCaves/P007" "ShoreCaves/P008")
# ENVS=("sqh/sqh_12_03_2024_02")
ENVS=("tartanevo_gtsam/tartanairv2")
# ENVS=("CoalMine/P000")
#ENVS=("CountryHouse/P000" "CountryHouse/P001" "CountryHouse/P002" "CountryHouse/P003" "CountryHouse/P004" "CountryHouse/P005" "MiddleEast/P000" "MiddleEast/P001" "MiddleEast/P002" "MiddleEast/P003"
#"ShoreCaves/P001" "ShoreCaves/P005" "CoalMine/P000")

TRAJS=("CountryHouse/P000")
# Loop over each environment and process the corresponding trajectories
for ENV in "${ENVS[@]}"; do
  for TRAJ in "${TRAJS[@]}"; do
    echo "Processing environment: $ENV"

    # Define the file paths
    # GROUND_TRUTH_TUM="$ENV/pose_lcam_front_tum.txt"
    GROUND_TRUTH_TUM="$ENV/$TRAJ/ground_truth.tum"
    DEVO_TUM="$ENV/$TRAJ/CountryHouse-P000-gtsam_isam2.tum"
    ZIP_DIR="$PLOTS_BASE/$ENV/$TRAJ"
    PLOT_DIR="$PLOTS_BASE/$ENV"

    mkdir -p $PLOTS_BASE/$ENV
    mkdir -p $PLOTS_BASE/$ENV/$TRAJ
    mkdir -p $PLOT_DIR/evo_traj
    

    # Run the evo_traj command
    if [ "$COMMAND" == "evo_traj" ]; then
      evo_traj tum "$DEVO_TUM" --ref="$GROUND_TRUTH_TUM" --save_plot="$PLOT_DIR"/evo_traj/"$TRAJ" --plot_mode=xy --t_max_diff=0.1 --align --correct_scale
    elif [ "$COMMAND" == "evo_rpe" ]; then
      evo_rpe tum "$GROUND_TRUTH_TUM" "$DEVO_TUM" --save_plot="$ZIP_DIR"_rpe_devo --align --correct_scale --t_max_diff=0.005 --delta=10.0 --delta_unit=m --pose_relation point_distance_error_ratio --all_pairs --save_results "$ZIP_DIR"_rpe_devo.zip
    elif [ "$COMMAND" == "evo_ape" ]; then
      evo_ape tum "$GROUND_TRUTH_TUM" "$DEVO_TUM" --save_plot="$ZIP_DIR"_ape_devo --align --correct_scale --t_max_diff=0.005 --save_results "$ZIP_DIR"_ape_devo.zip
    else
      echo "Invalid command: $COMMAND"
      exit 1
    fi

    echo "Finished processing environment: $ENV"
  done
  if [ "$COMMAND" == "evo_rpe" ]; then
      evo_res "$PLOT_DIR"/*_rpe_devo.zip --save_table "$PLOT_DIR"/results/rpe.csv
  elif [ "$COMMAND" == "evo_ape" ]; then
      evo_res "$PLOT_DIR"/*_ape_devo.zip --save_table "$PLOT_DIR"/results/ape.csv
  #else
  #    echo "Invalid command: $COMMAND"
  #    exit 1
  fi

done
