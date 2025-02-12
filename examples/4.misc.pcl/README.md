# C++ Sample: 4.misc.pcl

This project provides a set of examples and utilities for working with the Orbbec SDK and PCL (Point Cloud Library). It includes various modules for processing depth, color, and point cloud data.

## Project Structure

The project is organized into several subdirectories, each corresponding to different functionalities:

- **`pcl`**: Contains PCL-related functionality for processing point clouds and depth data.
- **`pcl_color`**: Contains PCL-related functionality specifically for handling color point clouds.

## Building the Project
You need to find the following option and set it to ON
```CMake
option(OB_BUILD_PCL "Build Point Cloud Library examples" ON)
```

1. If you compile the project from the OrbbecSDK source directory, you can find this option in the `option.cmake` file under the `cmake` folder.
2. If you compile the project using the Release package we provided, you can find this option in the `CMakeLists.txt` file under the `example` folder.