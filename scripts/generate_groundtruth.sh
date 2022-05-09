#! /bin/bash

> /home/norlab/Desktop/doctorat/icra2023/data/2022-05-05_MatingCallDay/theodolite.txt

yaw=(0 20 19 19 19 # 5
     19 17 17 17 17 # 10
     18 18 20 20 22 # 15
     22 22 22 24 25 # 20
     28 30 34 39 39 # 25
     43 43 43 44 44 # 30
     44 44 44 46 47 # 35
     47 47 47 44 51 # 40
     51 51 52 53 55 # 45
     58 58 61 63 64 # 50
     66 68 71 73 74 # 55
     76 78) # 57

x=(-0.5 0.5 0 0 1 # 5
   1.5 1.5 0 1 0.5 # 10
   1 1 1 1 1 # 15
   0 0 0 1 1 # 20
   1 1 1 1 1 # 25
   1 1 1 1 1.5 # 30
   1.5 1.5 1.5 1.5 2.5 # 35
   2.5 2.5 2.5 2 2 # 40
   2 2 2 2 2 # 45
   2 2 2.5 2 3 # 50
   3 3 3 3 3 # 55
   3 3) # 57

y=(-1.5 -2.5 -2.5 -2.5 -2.5 # 5
   -2.5 -2.5 -2 -2 -2.5 # 10
   -2 -2 -2 -2 -2 # 15
   -2 -2 -2 -1.5 -2 # 20
   -2 -2 -2 -2 -2 # 25
   -2 -2 -2 -2 -2 # 30
   -2 -2 -2 -2 -2 # 35
   -2 -2 -2 -1.5 -1.5 # 40
   -1.5 -1.5 -1.5 -1.5 -1.5 # 45
   -1.5 -1.5 -1 -1.5 -0.5 # 50
   -0.5 -0.5 -0.5 -0.5 -0.5 # 55
   0 0) # 57

for i in ${!yaw[@]}
do
  if [ ! -d /home/norlab/Desktop/doctorat/icra2023/data/2022-05-05_MatingCallDay/scans_$((i + 1)) ]
  then
    mkdir -p /home/norlab/Desktop/doctorat/icra2023/data/2022-05-05_MatingCallDay/scans_$((i + 1))
  fi

  roslaunch publi_deskewing_uncertainty cube_groundtruth.launch bagfile:=/home/norlab/Desktop/doctorat/icra2023/data/2022-05-05_MatingCallDay/run_$((i + 1)).bag final_transformation_file_name:=/home/norlab/Desktop/doctorat/icra2023/data/2022-05-05_MatingCallDay/theodolite.txt initial_yaw:=${yaw[$i]} initial_x:=${x[$i]} initial_y:=${y[$i]} scan_directory:=/home/norlab/Desktop/doctorat/icra2023/data/2022-05-05_MatingCallDay/scans_$((i + 1)) &

  sleep 3
  while [[ ! -z `pgrep mapper_node` ]]
  do
      sleep 1
  done
  killall rviz
  killall imu_odom_node
  killall pointcloud2_deskew_node
  killall cloud_node_stamped
  killall static_transform_publisher
  killall rosmaster
done

