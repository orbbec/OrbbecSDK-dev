#!/usr/bin/env bash

PROJECT_ROOT=$(git rev-parse --show-toplevel)

echo "Building openorbbecsdk for linux_x86_64"

# Detect the latest docker image with prefix "openorbbecsdk-env:x86_64_"
DOCKER_IMAGE=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "openorbbecsdk-env:x86_64_" | sort -r | head -n 1)
if [ -z "$DOCKER_IMAGE" ]; then
    echo "Docker image openorbbecsdk-env:x86_64 with latest timestamp not found"
    exit 1
fi

# Variables for version and timestamp
VERSION=$(grep -oP 'project\(\w+\s+VERSION\s+\d+\.\d+\.\d+' $PROJECT_ROOT/CMakeLists.txt | grep -oP '\d+\.\d+\.\d+')
TIMESTAMP=$(date +"%Y%m%d%H%M")

PLATFORM="linux_$(uname -m)"
echo "Building openorbbecsdk for $PLATFORM"
COMMIT_HASH=$(git rev-parse --short HEAD)
# Define the installation directory name with version, timestamp, and platform
INSTALL_DIR="openorbbecsdk_v${VERSION}_${TIMESTAMP}_${COMMIT_HASH}_${PLATFORM}"

# Create build directory
cd $PROJECT_ROOT
mkdir -p build-release-$PLATFORM
cd build-release-$PLATFORM || exit

# Create target directory for installation
mkdir -p $INSTALL_DIR

# Get the current user and group ID
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# Run cmake and build inside Docker container using custom entrypoint and user permissions
docker run --rm -u $USER_ID:$GROUP_ID -v "$PROJECT_ROOT":/workspace -w /workspace/build-release-$PLATFORM --entrypoint /bin/bash $DOCKER_IMAGE -c "
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DBUILD_SHARED_LIBS=ON || { echo 'Failed to run cmake'; exit 1; }
    make install -j\$(($(nproc) - 1)) || { echo 'Failed to build openorbbecsdk'; exit 1; }
    cd $INSTALL_DIR
    cd ..
    zip -rpy '${INSTALL_DIR}.zip' $INSTALL_DIR
"

# Move the compressed file to the project root
mv "${INSTALL_DIR}.zip" $PROJECT_ROOT
# remove the installation directory
rm -rf $INSTALL_DIR
echo "Done building and compressing openorbbecsdk for $PLATFORM"
echo "Done building openorbbecsdk for $PLATFORM"
