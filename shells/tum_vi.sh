#!/bin/bash
cd ..
currentDir=$(pwd)
runType=Monocular
datasetType=dataset-corridor2_512_16
timestampType=dataset-corridor2_512

pathDatasetTUM_VI='/home/sph/Downloads/dataset/TUM' #Example, it is necesary to change it by the dataset path
if  [ -d "$pathDatasetTUM_VI" ];then
  echo  ""
else
  echo  "该文件夹不存在 ${pathDatasetTUM_VI} "
  exit 1
fi

if  [ -f "$currentDir/Vocabulary/ORBvoc.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Vocabulary/ORBvoc.txt "
  exit 1
fi
if  [ -f "$currentDir/Examples/$runType/TUM_512.yaml" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/$runType/TUM_512.yaml "
  exit 1
fi
# 这里有个问题是我实际的路径是/media/sph/TOSHIBA EXT/dataset/dataset-corridor4_512_16/dataset-corridor4_512_16/mav0/cam0/data
# 而代码中看到的是/media/sph/TOSHIBA EXT/dataset/dataset-corridor4_512_16/mav0/cam0/data, 少了一层dataset-corridor4_512_16
# TODO: 要确认一下是代码bug还是什么问题
if  [ -d ""$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data" ];then
  echo  ""
else
  echo  "该文件夹不存在 "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data "
  exit 1
fi
if  [ -f "$currentDir/Examples/$runType/TUM_TimeStamps/$timestampType.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/$runType/TUM_TimeStamps/$timestampType.txt "
  exit 1
fi
if  [ -d "logs" ];then
  echo  ""
else
  echo  "logs 该文件夹不存在 创建"
  mkdir logs
fi

#------------------------------------
# Monocular Examples
echo "Launching Room 4 with $runType sensor"
./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM_512.yaml "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data Examples/Monocular/TUM_TimeStamps/$timestampType.txt "$timestampType_mono"

#------------------------------------
# Stereo Examples
#echo "Launching Room 4 with Stereo sensor"
#./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room4_512.txt dataset-room4_512_stereo

#------------------------------------
# Monocular-Inertial Examples
#echo "Launching Room 4 with $runType sensor"
#./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/"$runType"/TUM_512.yaml "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data Examples/"$runType"/TUM_TimeStamps/dataset-room4_512.txt Examples/"$runType"/TUM_IMU/dataset-room4_512.txt dataset-room4_512_monoi

# MultiSession Monocular Examples
#echo "Launching all Rooms (1-6) in the same session with Stereo-Inertial sensor"
#./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room2_512.txt "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room3_512.txt "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room4_512.txt "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room5_512.txt "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-rooms123456_monoi

#------------------------------------
# Stereo-Inertial Examples
#echo "Launching Room 4 with $runType sensor"
#./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room4_512.txt dataset-room4_512_stereoi

# MultiSession Stereo-Inertial Examples
#echo "Launching all Rooms (1-6) in the same session with Stereo-Inertial sensor"
#./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room2_512.txt "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room3_512.txt "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam0/data "$pathDatasetTUM_VI"/"$datasetType"/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room4_512.txt "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room5_512.txt "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-rooms123456_stereoi
