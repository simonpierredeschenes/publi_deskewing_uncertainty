#! /bin/bash

if [ $# -ne 2 ]
then
  echo "Incorrect number of arguments! Argument 1 is the path to the bag file. Argument 2 is a boolean indicating whether or not to use interpolated measurements."
  exit 1
fi

bag_file=$1
use_interpolated_measurements=$2

if [ $use_interpolated_measurements = "true" ]
then
  ros2 run imu_saturation imu_publisher_node --ros-args -r imu_in:=/MTI30_imu/data -r imu_out:=/MTI30_imu/data_interpolated -p imu_measurements_file_name:="$bag_file"_xsens.csv -p saturation_point:=10.5 &
  ros2 run utility_nodes waiting_node --ros-args -r topic_in:=imu_publisher_status
  ros2 launch publi_deskewing_uncertainty cube_launch.xml bagfile:=$bag_file use_interpolated_measurements:=true
elif [ $use_interpolated_measurements = "false" ]
then
  ros2 launch publi_deskewing_uncertainty cube_launch.xml bagfile:=$bag_file use_interpolated_measurements:=false
else
  echo "Invalid value for second argument. Possible values are true or false."
  exit 1
fi

exit 0

