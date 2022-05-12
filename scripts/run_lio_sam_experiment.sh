#! /bin/bash

if [ $# -ne 2 ]
then
  echo "Incorrect number of arguments! Argument 1 is the folder containing the bag files. Argument 2 is the folder in which to store the results."
  exit
fi

data_folder=$1
results_folder=$2

if [ ! -d $results_folder ]
then
        mkdir -p $results_folder
fi
matrix_file="$results_folder"/mapper.txt
> $matrix_file

for run_bag in `ls -v "$data_folder"/run*.bag`
do
  roslaunch publi_deskewing_uncertainty lio_sam.launch bagfile:=$run_bag final_transformation_file_name:=$matrix_file &

  sleep 3
  while [[ ! -z `pgrep play` ]]
  do
      sleep 1
  done
  sleep 10


  killall rviz
  killall lio_sam_featureExtraction
  killall lio_sam_imageProjection
  killall lio_sam_imuPreintegration
  killall lio_sam_mapOptmization
  killall lidar_adapter_node
  rosnode kill /lio_sam_transformation_exporter_node
  killall cloud_node_stamped
  killall static_transform_publisher
  killall rosmaster
done

