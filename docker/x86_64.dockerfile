FROM ubuntu:18.04

RUN sed -i 's|http://security.ubuntu.com/ubuntu|http://mirrors.aliyun.com/ubuntu|g' /etc/apt/sources.list

# Builder dependencies installation
RUN apt-get update -o Acquire::http::proxy=false -o Acquire::https::proxy=false \
    && apt-get install -qq -y --no-install-recommends -o Acquire::http::proxy=false -o Acquire::https::proxy=false \
    build-essential \
    cmake \
    git \
    openssl \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    zip \
    && rm -rf /var/lib/apt/lists/*
