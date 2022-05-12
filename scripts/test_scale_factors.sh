#! /bin/bash

rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_scale_01 0.1
rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_scale_05 0.5
rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_scale_1 1
rosrun publi_deskewing_uncertainty run_cube_experiment.sh /home/guest/data/2022-05-05_MatingCallDay_processed/ /home/guest/results/new_dataset_gtplane_scale_10 10

