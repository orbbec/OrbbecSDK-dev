# target options
option(OB_BUILD_EXAMPLES "Build SDK examples" ON)
option(OB_BUILD_TESTS "Build tests" OFF)
option(OB_BUILD_TOOLS "Build tools" OFF)
option(OB_BUILD_DOCS "Build api document and install doc" ON)

# platform options
option(OB_BUILD_ANDROID "Build Android " OFF)
option(OB_BUILD_LINUX_ARM64 "Build linux arm64, will use ARM64 toolchains on linux platform " OFF)
option(OB_BUILD_LINUX_ARM32 "Build linux arm32, will use ARM32 toolchains on linux platform" OFF)

# component options
option(OB_BUILD_USB_PORT "Enable this to support USB/UVC/HID communication" ON)
option(OB_BUILD_NET_PORT "Enable this to support network/GVCP/RTSP communication" OFF)

# install options
option(OB_INSTALL_FILTER_DEV_HEADERS "Install HEADER files for filter development" OFF)
