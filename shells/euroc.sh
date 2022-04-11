#!/bin/bash
cd ..
currentDir=$(pwd)
runType=Monocular
datasetType=MH_01_easy
timestampType=MH01

pathDatasetEuroc='/EuRoC' #Example, it is necesary to change it by the dataset path
if  [ -d "$pathDatasetEuroc" ];then
  echo  ""
else
  echo  "该文件夹不存在 ${pathDatasetEuroc} "
  exit 1
fi

if  [ -f "$currentDir/Vocabulary/ORBvoc.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Vocabulary/ORBvoc.txt "
  exit 1
fi
if  [ -f "$currentDir/Examples/$runType/EuRoC.yaml" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/$runType/EuRoC.yaml "
  exit 1
fi
if  [ -d $pathDatasetEuroc/$datasetType ];then
  echo  ""
else
  echo  "该文件夹不存在 $pathDatasetEuroc/$datasetType "
  exit 1
fi
if  [ -f "$currentDir/Examples/$runType/EuRoC_TimeStamps/$timestampType.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/$runType/EuRoC_TimeStamps/$timestampType.txt "
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
echo "Launching $timestampType with $runType sensor"
./Examples/$runType/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/$runType/EuRoC.yaml $pathDatasetEuroc/$datasetType ./Examples/$runType/EuRoC_TimeStamps/$timestampType.txt "$datasetType_$runType"