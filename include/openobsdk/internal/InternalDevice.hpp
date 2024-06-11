#pragma once

#include <memory>
#include <string>
#include <vector>

#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/hpp/Types.hpp"
#include "libobsensor/h/Property.h"

namespace ob {

class OB_EXTENSION_API InternalDevice : public Device {
public:
    InternalDevice(Device &&baseDevice);
    virtual ~InternalDevice() noexcept;

    // 在这里增加内部使用的接口
};

}  // namespace ob