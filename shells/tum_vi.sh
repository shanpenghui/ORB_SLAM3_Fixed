#!/bin/bash
currentDir=$(pwd)

pathDatasetTUM_VI='/media/sph/TOSHIBA EXT/dataset' #Example, it is necesary to change it by the dataset path
if  [ -d "$pathDatasetTUM_VI" ];then
  echo  ""
else
  echo  "该文件夹不存在 ${pathDatasetTUM_VI} "
fi

#------------------------------------
# Monocular Examples
#echo "Launching Room 4 with Monocular sensor"
#./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM_VI.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data Examples/Monocular/TUM_TimeStamps/dataset-room4_512.txt dataset-room4_512_mono

#------------------------------------
# Stereo Examples
#echo "Launching Room 4 with Stereo sensor"
#./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo/TUM_VI.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-room4_512.txt dataset-room4_512_stereo

#------------------------------------
# Monocular-Inertial Examples
echo "Launching Room 4 with Monocular-Inertial sensor"
if  [ -f "$currentDir/Vocabulary/ORBvoc.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Vocabulary/ORBvoc.txt "
fi
if  [ -f "$currentDir/Examples/Monocular-Inertial/TUM_512.yaml" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/Monocular-Inertial/TUM_512.yaml "
fi
# 这里有个问题是我实际的路径是/media/sph/TOSHIBA EXT/dataset/dataset-corridor4_512_16/dataset-corridor4_512_16/mav0/cam0/data
# 而代码中看到的是/media/sph/TOSHIBA EXT/dataset/dataset-corridor4_512_16/mav0/cam0/data, 少了一层dataset-corridor4_512_16
# TODO: 要确认一下是代码bug还是什么问题
if  [ -d ""$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data" ];then
  echo  ""
else
  echo  "该文件夹不存在 "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "
fi
if  [ -f "$currentDir/Examples/Monocular-Inertial/TUM_TimeStamps/dataset-room4_512.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/Monocular-Inertial/TUM_TimeStamps/dataset-room4_512.txt "
fi
./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data Examples/Monocular-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Monocular-Inertial/TUM_IMU/dataset-room4_512.txt dataset-room4_512_monoi

# MultiSession Monocular Examples
#echo "Launching all Rooms (1-6) in the same session with Stereo-Inertial sensor"
#./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room2_512.txt "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room3_512.txt "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room4_512.txt "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room5_512.txt "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-rooms123456_monoi

#------------------------------------
# Stereo-Inertial Examples
#echo "Launching Room 4 with Stereo-Inertial sensor"
#./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room4_512.txt dataset-room4_512_stereoi

# MultiSession Stereo-Inertial Examples
#echo "Launching all Rooms (1-6) in the same session with Stereo-Inertial sensor"
#./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room2_512.txt "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room3_512.txt "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room4_512.txt "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room5_512.txt "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-rooms123456_stereoi
