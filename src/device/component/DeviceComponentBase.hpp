#pragma once

#include "IDeviceComponent.hpp"
#include "IDevice.hpp"
#include "exception/ObException.hpp"

#include <memory>

namespace libobsensor {

class DeviceComponentBase : virtual public IDeviceComponent {
public:
    DeviceComponentBase(std::shared_ptr<IDevice> owner) : owner_(owner) {}

    std::shared_ptr<IDevice> getOwner() const override {
        auto device = owner_.lock();
        if(!device) {
            throw camera_disconnected_exception("Device is disconnected or has been destroyed");
        }
        return owner_.lock();
    }

private:
    std::weak_ptr<IDevice> owner_;
};

}  // namespace libobsensor