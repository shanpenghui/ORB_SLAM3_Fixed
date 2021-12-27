echo "Configuring and building Thirdparty/DBoW2 ..."

cd ../Thirdparty/DBoW2
rm -rf build lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 8

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
rm -rf build lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 8

echo "Uncompress vocabulary ..."

cd ../../../Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 8

echo "Building ROS2 nodes"

source /opt/ros/foxy/setup.bash

cd ../Examples/ROS2
currentDir=$(pwd)

cd ORB_SLAM3_WS
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro=foxy
colcon build --symlink-install
