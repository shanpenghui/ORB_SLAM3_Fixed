#!/bin/bash

cd ..
currentDir=$(pwd)
# 配置运行类型
runType=RGB-D
# 配置文件夹
datasetType=rgbd_dataset_freiburg1_desk

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
if  [ -f "$currentDir/Examples/$runType/TUM1.yaml" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/$runType/TUM1.yaml "
  exit 1
fi

if  [ -d ""$pathDatasetTUM_VI"/"$datasetType"" ];then
  echo  ""
else
  echo  "该文件夹不存在 "$pathDatasetTUM_VI"/"$datasetType" "
  exit 1
fi
if  [ -f "$currentDir/Examples/$runType/associations/fr1_desk.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/$runType/associations/fr1_desk.txt "
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
echo "Launching $datasetType with $runType SLAM"
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/"$runType"/TUM1.yaml "$pathDatasetTUM_VI"/"$datasetType" "$currentDir"/Examples/"$runType"/associations/fr1_desk.txt
