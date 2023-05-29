#! /bin/bash

if [ $# -ne 3 ]
then
  echo "Incorrect number of arguments! Argument 1 is the folder containing the bag files. Argument 2 is the folder in which to store the results. Argument 3 is the scale factor."
  exit
fi

data_folder=$1
results_folder=$2
scale_factor=$3

if [ ! -d $results_folder ]
then
        mkdir -p $results_folder
fi
matrix_file="$results_folder"/mapper.txt
> $matrix_file

for run_bag in `ls -v "$data_folder"/run*.bag`
do
  roslaunch publi_deskewing_uncertainty cube_launch.xml bagfile:=$run_bag final_transformation_file_name:=$matrix_file use_icra_model:=true scale_factor:=$scale_factor &
  sleep 3
  while [[ ! -z `pgrep mapper_node` ]]
  do
      sleep 1
  done
  killall rviz2
  killall imu_odom_node
  killall pointcloud2_deskew_node
  killall cloud_node_stamped
  killall static_transform_publisher
done

exit 0

