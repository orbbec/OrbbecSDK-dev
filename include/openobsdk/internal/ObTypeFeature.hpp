#pragma once
#include "ObTypeHelper.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"

// SDK 内部不可使用
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const ob::StreamProfile &profile);
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const ob::VideoStreamProfile &profile);

namespace ob {
template <typename T> std::string obTypeToString(T dataType) {
    std::ostringstream oss;
    oss << dataType;
    return oss.str();
}
}  // namespace ob
