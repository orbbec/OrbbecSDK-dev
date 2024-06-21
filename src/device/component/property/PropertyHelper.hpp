#pragma once
#include "openobsdk/h/Property.h"
#include <map>
#include <string>

namespace libobsensor {
struct OBPropertyBaseInfo {
    OBPropertyType type;
    const char    *name;
};

extern const std::map<OBPropertyID, OBPropertyBaseInfo> OBPropertyBaseInfoMap;

}  // namespace libobsensor