#pragma once

#include "ISourcePort.hpp"

namespace libobsensor {

// todo: complete this class
class IDevice {
public:
    virtual ~IDevice() = default;
};

}  // namespace libobsensor


#ifdef __cplusplus
extern "C" {
#endif
struct ob_device_t {
    std::shared_ptr<libobsensor::IDevice> device;
};
#ifdef __cplusplus
}
#endif