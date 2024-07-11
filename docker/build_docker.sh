#!/usr/bin/env bash

# Prompt the user to select architecture
echo "Please select the architecture:"
echo "1) x86_64"
echo "2) aarch64"
read -p "Enter your choice (1 or 2): " choice

# Set the ARCH variable based on user input
case $choice in
    1)
        ARCH="x86_64"
        ;;
    2)
        ARCH="aarch64"
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

# Set the DATE variable
DATE=$(date +'%Y%m%d')

# Build the Docker image
docker build \
    -f ${ARCH}.dockerfile . -t openorbbecsdk-env:${ARCH}_${DATE} \
    --label ade_image_commit_sha="$(git rev-parse HEAD)" \
    --label ade_image_commit_tag="$(date +'%Y%m%d.%H%M%S')"
