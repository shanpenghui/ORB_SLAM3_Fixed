#!/bin/bash
cd ..
currentDir=$(pwd)
runType=Monocular-Inertial

pathDatasetEuroc='/home/shanph/disk/dataset/EuRoC' #Example, it is necesary to change it by the dataset path
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
if  [ -d ""$pathDatasetEuroc"/MH_01_easy" ];then
  echo  ""
else
  echo  "该文件夹不存在 "$pathDatasetEuroc"/MH_01_easy "
  exit 1
fi
if  [ -f "$currentDir/Examples/$runType/EuRoC_TimeStamps/MH01.txt" ];then
  echo  ""
else
  echo  "该文件不存在 $currentDir/Examples/$runType/EuRoC_TimeStamps/MH01.txt "
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
echo "Launching MH01 with $runType sensor"
./Examples/"$runType"/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/"$runType"/EuRoC.yaml "$pathDatasetEuroc"/MH_01_easy ./Examples/"$runType"/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

#------------------------------------
# Stereo Examples
#echo "Launching MH01 with Stereo sensor"
#./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

#------------------------------------
# Monocular-Inertial Examples
#echo "Launching MH01 with $runType sensor"
#./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_01_easy ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

# MultiSession Monocular Examples
#echo "Launching Machine Hall with Monocular-Inertial sensor"
#./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuroc"/MH02 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuroc"/MH03 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuroc"/MH04 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuroc"/MH05 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH05.txt dataset-MH01_to_MH05_monoi

#------------------------------------
# Stereo-Inertial Examples
#echo "Launching MH01 with Stereo-Inertial sensor"
#./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi

# MultiSession Stereo-Inertial Examples
#echo "Launching Machine Hall with Stereo-Inertial sensor"
#./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuroc"/MH02 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuroc"/MH03 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuroc"/MH04 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuroc"/MH05 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH05.txt dataset-MH01_to_MH05_stereoi
