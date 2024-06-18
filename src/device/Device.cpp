#include "Device.hpp"

#include "context/Context.hpp"

namespace libobsensor {

Device::Device() : ctx_(Context::getInstance()) {}

}  // namespace libobsensor