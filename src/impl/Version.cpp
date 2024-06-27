#include "libobsensor/h/version.h"
#include "ImplTypes.hpp"
#include "exception/ObException.hpp"

#include "environment/Version.hpp"

#ifdef __cplusplus
extern "C" {
#endif

int ob_get_version() {
    return OB_API_VERSION;
}

int ob_get_major_version() {
    return OB_API_MAJOR_VERSION;
}

int ob_get_minor_version() {
    return OB_API_MINOR_VERSION;
}

int ob_get_patch_version() {
    return OB_API_PATCH_VERSION;
}

const char *ob_get_stage_version() {
    return OB_API_STAGE_VERSION;
}

#ifdef __cplusplus
}
#endif