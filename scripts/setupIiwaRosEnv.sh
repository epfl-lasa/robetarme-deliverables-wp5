#!/usr/bin/bash
# This script sets up the environment for the use of iiwa_ros in the project

checkError () {
    if [ $? -ne 0 ]; then
    echo "An Error occured, setup aborted."
    exit 1
    fi
}

mkdir temp
cd temp

# KUKA FRI
echo "* CLONING LIBS KUKA..."
git clone --depth 1 --branch v0.1.0 git@github.com:epfl-lasa/kuka_fri.git
cd kuka_fri
echo "* CONFIGURING LIBS KUKA..."
./waf configure 2>&1 >/dev/null
./waf 2>&1 >/dev/null
checkError
echo "* INSTALLING LIBS KUKA (sudo)..."
sudo ./waf install 2>&1 >/dev/null
cd ..

# SpaceVecAlg
echo "* CLONING SpaceVecAlg..."
git clone --recursive --depth 1 --branch v1.2.6 git@github.com:jrl-umi3218/SpaceVecAlg.git
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF ..
make -j ${NB_CPU_THREAD} 2>&1 >/dev/null
checkError
echo "* INSTALLING SpaceVecAlg..."
sudo make install 2>&1 >/dev/null
cd ../..

# RBDyn
echo "* CLONING RBDyn..."
git clone --recursive --depth 1 --branch v1.8.3 git@github.com:jrl-umi3218/RBDyn.git
cd RBDyn
echo "* BUILDING RBDyn..."
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF ..
make -j ${NB_CPU_THREAD} 2>&1 >/dev/null
checkError
echo "* INSTALLING RBDyn..."
sudo make install 2>&1 >/dev/null
cd ../..

# mc_rbdyn_urdf
echo "* CLONING mc_rbdyn_urdf..."
git clone --recursive git@github.com:jrl-umi3218/mc_rbdyn_urdf.git
cd mc_rbdyn_urdf
git checkout c7eb72b46b3753ed3542793778e0cc59496574ef
echo "* BUILDING mc_rbdyn_urdf..."
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF ..
make -j ${NB_CPU_THREAD} 2>&1 >/dev/null
checkError
echo "* INSTALLING mc_rbdyn_urdf..."
sudo make install 2>&1 >/dev/null
cd ../..

# Corrade
echo "* CLONING Corrade..."
git clone git@github.com:mosra/corrade.git
cd corrade
git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5
echo "* BUILDING Corrade..."
mkdir build && cd build
cmake ..
make -j ${NB_CPU_THREAD} 2>&1 >/dev/null
checkError
echo "* INSTALLING Corrade..."
sudo make install 2>&1 >/dev/null
cd ../..

# robot_controllers
echo "* CLONING robot_controllers..."
git clone git@github.com:epfl-lasa/robot_controllers.git
cd robot_controllers
git checkout 727bf85aaca675a41c281e830149e023154efe14
echo "* BUILDING robot_controllers..."
mkdir build && cd build
cmake ..
make -j ${NB_CPU_THREAD} 2>&1 >/dev/null
checkError
echo "* INSTALLING robot_controllers..."
sudo make install 2>&1 >/dev/null
cd ../..

# Clean up
cd ..
echo "* CLEANING WORKSPACE..."
rm -rf temp

echo "* INSTALLING ROS DEPENDENCIES..."
sudo apt install -y ros-noetic-ros-control ros-noetic-ros-controllers

