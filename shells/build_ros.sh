echo "Building ROS nodes"

source /opt/ros/melodic/setup.bash

cd ../Examples/ROS
currentDir=$(pwd)

cd ORB_SLAM3
rm -rf build lib
mkdir build

cd build

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$currentDir
cmake ..
make -j 4
