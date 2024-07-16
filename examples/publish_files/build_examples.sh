#!/bin/bash

# update apt-get
sudo apt-get update

#install compiler and tools
echo "Installing compiler and tools..."
sudo apt-get install -y build-essential cmake git

#install dependencies
echo "Installing dependencies..."
sudo apt-get install -y libopencv-dev

# restore current directory
current_dir=$(pwd)

# cd to the directory where this script is located
cd "$(dirname "$0")"
project_dir=$(pwd)
examples_dir=$project_dir/examples

#cmake
echo "Building examples..."
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release $examples_dir
make -j4

# copy the executable files to the project directory
cp bin/* $project_dir/bin

# clean up
cd $project_dir
rm -rf build

echo "OpenOrbbecSDK examples built successfully!"
echo "The executable files located in: $project_dir/bin"

cd $current_dir