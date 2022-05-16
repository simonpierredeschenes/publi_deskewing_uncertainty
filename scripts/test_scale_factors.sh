#! /bin/bash

rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_noise_001 0.01
rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_noise_003 0.03
rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_noise_01 0.1
rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_noise_03 0.3

