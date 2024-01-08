#! /bin/bash

if [ $# -ne 3 ]
then
  echo "Incorrect number of arguments! Argument 1 is the folder containing the bag files. Argument 2 is the folder in which to store the results. Argument 3 is a boolean indicating whether or not to use interpolated measurements."
  exit
fi

data_folder=$1
results_folder=$2
use_interpolated_measurements=$3

if [ ! -d $results_folder ]
then
        mkdir -p $results_folder
fi
matrix_file="$results_folder"/mapper.txt
> $matrix_file

for file in `ls -v "$data_folder"`
do
  if [[ -f "$data_folder/$file/metadata.yaml" && $file = run_* ]]
  then
    run_nb=`echo $file | grep -o "[0-9]*"`
    run_bag="$data_folder/$file"

    if [ $use_interpolated_measurements = "true" ]
    then
      ros2 run imu_saturation imu_publisher_node --ros-args -r imu_in:=/MTI30_imu/data -r imu_out:=/MTI30_imu/data_interpolated -p imu_measurements_file_name:="$run_bag"_xsens.csv -p saturation_point:=10.5 &
      ros2 run utility_nodes waiting_node --ros-args -r topic_in:=imu_publisher_status
      ros2 launch publi_deskewing_uncertainty cube_launch.xml bagfile:=$run_bag final_transformation_file_name:=$matrix_file use_interpolated_measurements:=true imu_measurements_file_name:="$results_folder"/speeds_imu_"$run_nb".csv icp_measurements_file_name:="$results_folder"/speeds_icp_"$run_nb".csv final_map_file_name:="$results_folder"/map_"$run_nb".vtk final_trajectory_file_name:="$results_folder"/trajectory_"$run_nb".vtk &
    else
      ros2 launch publi_deskewing_uncertainty cube_launch.xml bagfile:=$run_bag final_transformation_file_name:=$matrix_file use_interpolated_measurements:=false imu_measurements_file_name:="$results_folder"/speeds_imu_"$run_nb".csv icp_measurements_file_name:="$results_folder"/speeds_icp_"$run_nb".csv final_map_file_name:="$results_folder"/map_"$run_nb".vtk final_trajectory_file_name:="$results_folder"/trajectory_"$run_nb".vtk &
    fi
    sleep 10

    ros2 node list > /dev/null # force node discovery
    while [[ `ros2 node info /mapper_node 2>&1` != "Unable to find node '/mapper_node'" ]]
    do
        sleep 1
    done

    killall rviz2
    killall angular_velocity_logger_node
    killall imu_odom_node
    killall imu_filter_madgwick_node
    killall pointcloud2_deskew_node
    killall static_transform_publisher
    killall imu_publisher_node
    killall ros2
  fi
done

exit 0

