#pragma once

#include "IDevice.hpp"

namespace libobsensor {

class Context;

class Device : public IDevice {
public:
    Device();
    virtual ~Device() noexcept = default;

    // todo: move base function implementation to here

private:
    std::shared_ptr<Context> ctx_;  // handle the lifespan of the context
};

}  // namespace libobsensor