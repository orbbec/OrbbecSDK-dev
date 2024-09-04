#!/usr/bin/env bash

CURRNET_DIR=$(pwd)

SCRIPT_DIR=$(dirname "$0")
cd $SCRIPT_DIR/../../

PROJECT_ROOT=$(pwd)

# Define the platform
arch=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    PLATFORM=linux_arm64
elif [ "$ARCH" = "armv7l" ]; then
    PLATFORM=linux_arm32
else
    PLATFORM=linux_$arch
fi

echo "Building openorbbecsdk for $PLATFORM"

# Variables for version and timestamp
VERSION=$(grep -oP 'project\(\w+\s+VERSION\s+\d+\.\d+\.\d+' $PROJECT_ROOT/CMakeLists.txt | grep -oP '\d+\.\d+\.\d+')
TIMESTAMP=$(date +"%Y%m%d%H%M")

git config --global --add safe.directory /workspace/OpenOrbbecSDK
COMMIT_HASH=$(git rev-parse --short HEAD)
PACKAGE_NAME="openorbbecsdk_${VERSION}_${TIMESTAMP}_${COMMIT_HASH}_${PLATFORM}"

# Create build directory
rm -rf build_$PLATFORM
mkdir -p build_$PLATFORM
cd build_$PLATFORM || exit

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

echo "Done building and compressing openorbbecsdk for $PLATFORM"
echo "Done building openorbbecsdk for $PLATFORM"

cd $CURRNET_DIR
