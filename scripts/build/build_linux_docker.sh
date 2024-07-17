#!/bin/bash

CURRNET_DIR=$(pwd)
PROJECT_ROOT=$(git rev-parse --show-toplevel)

# get arch from arg
ARCH=$1

if [ "$ARCH" == "" ]; then
    ARCH=$(uname -m)
fi

if [ "$ARCH" != "x86_64" ] && [ "$ARCH" != "aarch64" ] && [ "$ARCH" != "arm64" ]  && [ "$ARCH" != "arm" ]; then
    echo "Invalid architecture: $ARCH, supported architectures are x86_64, aarch64, and arm"
    exit 1
fi

platform="linux/$ARCH"
if [ "$ARCH" == "x86_64" ]; then
    platform="linux/amd64"
elif [ "$ARCH" == "aarch64" ]; then
    platform="linux/arm64"
fi

echo "Building openorbbecsdk for linux $ARCH via docker"

# detect docker image
if [ "$(docker images -q openorbbecsdk-env.$ARCH 2>/dev/null)" == "" ]; then
    echo "Docker image openorbbecsdk-env.$ARCH not found, building it with $PROJECT_ROOT/scripts/docker/$ARCH.dockerfile"
    cd $PROJECT_ROOT/scripts/docker

    # if docker buildx is not installed, install it
    if [ "$(docker buildx ls | grep default)" == "" ]; then
        echo "Installing docker buildx"
        docker buildx install
    fi

    # if docker buildx plarform does not exist, create it
    if [ "$(docker buildx ls | grep $platform)" == "" ]; then
        echo "Creating docker buildx platform $platform"
        docker run --privileged --rm tonistiigi/binfmt --install all
    fi

    # build docker image
    docker buildx build --platform $platform -t openorbbecsdk-env.$ARCH -f $PROJECT_ROOT/scripts/docker/$ARCH.dockerfile . --load || {
        echo "Failed to build docker image openorbbecsdk-env.$ARCH"
        exit 1
    }
    cd $CURRNET_DIR
fi

USER_ID=$(id -u)
GROUP_ID=$(id -g)

# run docker container and build openorbbecsdk
docker run --rm -u $USER_ID:$GROUP_ID -v $PROJECT_ROOT:/workspace -w /workspace --name OpenOrbbecSDK_Build_Liunx_$ARCH -it openorbbecsdk-env.$ARCH /bin/bash -c "cd /workspace && bash ./scripts/build/build_linux.sh"

echo "Done building openorbbecsdk for linux $ARCH via docker"
