if(OB_BUILD_PCL)
    # todo: require PCL, should check if PCL is installed, and if not, download and build it
    add_subdirectory(pcl)
    add_subdirectory(pcl_color)
endif()