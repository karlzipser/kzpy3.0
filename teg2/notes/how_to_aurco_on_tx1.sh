#!/bin/bash

###### Update dependencies if not on board
sudo add-apt-repository universe
sudo apt-get update
# Some general development libraries
sudo apt-get -y install build-essential make cmake cmake-curses-gui g++
# libav video input/output development libraries
sudo apt-get -y install libavformat-dev libavutil-dev libswscale-dev
# Video4Linux camera development libraries
sudo apt-get -y install libv4l-dev
# Eigen3 math development libraries
sudo apt-get -y install libeigen3-dev
# OpenGL development libraries (to allow creating graphical windows)
sudo apt-get -y install libglew1.6-dev
# GTK development libraries (to allow creating graphical windows)
sudo apt-get -y install libgtk2.0-dev


###### Create a folder which can easily hold up to 3-4 gb
###### and check out source 
mkdir /media/ubuntu/rosbags/opencv
cd /media/ubuntu/rosbags/opencv
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

cd opencv
mkdir build
cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DWITH_CUDA=ON -DCUDA_ARCH_BIN="3.2" -DCUDA_ARCH_PTX="" -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DCMAKE_BUILD_TYPE=RELEASE -DWITH_CUBLAS=ON -DENABLE_FAST_MATH=ON -DCUDA_FAST_MATH=ON ..
make -j4
sudo make install

