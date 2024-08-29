#!/usr/bin/env bash

CURRNET_DIR=$(pwd)

SCRIPT_DIR=$(dirname "$0")
cd $SCRIPT_DIR/../../

PROJECT_ROOT=$(pwd)

# Define the platform
arch=$(uname -m)
if [ "$arch" = "aarch64" ]; then
    platform=linux_arm64
elif [ "$arch" = "armv7l" ]; then
    platform=linux_arm32
else
    platform=linux_$arch
fi
# PLATFORM="linux_$ARCH"
# PLATFORM="linux_arm64"
echo "Building openorbbecsdk for $platform"

# Variables for version and timestamp
VERSION=$(grep -oP 'project\(\w+\s+VERSION\s+\d+\.\d+\.\d+' $PROJECT_ROOT/CMakeLists.txt | grep -oP '\d+\.\d+\.\d+')
TIMESTAMP=$(date +"%Y%m%d%H%M")

git config --global --add safe.directory /workspace/OpenOrbbecSDK
COMMIT_HASH=$(git rev-parse --short HEAD)
PACKAGE_NAME="openorbbecsdk_${VERSION}_${TIMESTAMP}_${COMMIT_HASH}_${platform}"

# Create build directory
rm -rf build_$platform
mkdir -p build_$platform
cd build_$platform || exit

# Create target directory for installation
INSTALL_DIR=$(pwd)/install/$PACKAGE_NAME
mkdir -p $INSTALL_DIR

# Build and install openorbbecsdk
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DOB_BUILD_EXAMPLES=OFF || { echo 'Failed to run cmake'; exit 1; }
make install -j8 || { echo 'Failed to build openorbbecsdk'; exit 1; }

# Compress the installation directory
cd $INSTALL_DIR
cd ..
zip -rpy ${PACKAGE_NAME}.zip ${PACKAGE_NAME} || { echo 'Failed to compress installation directory'; exit 1; }

echo "Done building and compressing openorbbecsdk for $platform"
echo "Done building openorbbecsdk for $platform"

cd $CURRNET_DIR
