#! /bin/bash

data_folder="/home/sp/data/icra2024/TIGS"

> "$data_folder"/gt_localization.txt

x=(-1.5 -1.5 -1.5 -1.5 -1.5 # 5
     -1.5 -2 -1.5 -1.5 -1.5 # 10
     -1.5 -2 -1.5 -2 -2 # 15
     -2 -2.5 -2 -2 -2 # 20
     -2 -2.25 -2 -2.5 -2 # 25
     -2 -2.25 -2.25 -2 -2 # 30
     -2.5 -2) # 32

y=(0.5 1 1 0.5 0.5 # 5
     1 1 1.5 0.5 1 # 10
     0.5 0 0 -0.5 -0.5 # 15
     -0.5 -0.5 -0.5 0 -0.5 # 20
     -0.5 -0.5 -1 -0.5 -1 # 25
     -0.5 -1 0 -1 -0.5 # 30
     -1 -1) # 32

yaw=(349.5 339 333 341 346 # 5
     339.5 349 334 347 332.5 # 10
     345 354 356 2.5 354 # 15
     357 357 2 353.5 355.5 # 20
     354.5 0 3 350 2.5 # 25
     347 5 345 4.5 6.5 # 30
     0 6.5) # 32

for i in ${!yaw[@]}
do
  if [ ! -d "$data_folder"/scans_$((i + 1)) ]
  then
    mkdir -p "$data_folder"/scans_$((i + 1))
  fi

  ros2 run imu_saturation imu_publisher_node --ros-args -r imu_in:=/MTI30_imu/data -r imu_out:=/MTI30_imu/data_interpolated -p imu_measurements_file_name:="$data_folder"/run_$((i + 1))_xsens.csv -p saturation_point:=10.5 &
  ros2 run utility_nodes waiting_node --ros-args -r topic_in:=imu_publisher_status
  ros2 launch publi_deskewing_uncertainty cube_groundtruth_launch.xml bagfile:="$data_folder"/run_$((i + 1))/ final_transformation_file_name:="$data_folder"/gt_localization.txt initial_yaw:=${yaw[$i]} initial_x:=${x[$i]} initial_y:=${y[$i]} scan_directory:="$data_folder"/scans_$((i + 1)) initial_map_file_name:="$data_folder"/gt_map.vtk &

  sleep 3
  while [[ ! -z `pgrep mapper_node` ]]
  do
      sleep 1
  done

  killall rviz2
  killall imu_odom_node
  killall imu_filter_madgwick_node
  killall pointcloud2_deskew_node
  killall static_transform_publisher
  killall imu_publisher_node
  killall ros2
done

