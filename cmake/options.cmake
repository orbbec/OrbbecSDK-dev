option(BUILD_SHARED_LIBS "Build shared libraries of the SDK" ON)

option(BUILD_EXAMPLES "Build SDK examples" ON)
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_TOOLS "Build tools" OFF)
option(BUILD_DOCS "Build api document and install doc" ON)

option(BUILD_USB_PORT "build USB source ports" ON)
option(BUILD_NET_PORT "build NET source ports" ON)

option(BUILD_ANDROID "Build Android " OFF)
option(BUILD_LINUX_ARM64 "Build linux arm64, will use ARM64 toolchains on linux platform " OFF)
option(BUILD_LINUX_ARM32 "Build linux arm32, will use ARM32 toolchains on linux platform" OFF)

option(INSTALL_MODULE_CORE "Install core module" OFF)
option(INSTALL_MODULE_SHARED "Install shared module" OFF)
option(INSTALL_MODULE_FILTER "Install filter module" OFF)