#pragma once

#include "IDevice.hpp"

#include <memory>
#include <map>

namespace libobsensor {

class Context;

class DeviceBase : public IDevice, public std::enable_shared_from_this<IDevice> {
public:
    DeviceBase();
    virtual ~DeviceBase() noexcept = default;

    void registerComponent(const std::string &name, std::shared_ptr<IDeviceComponent> component, bool lockRequired = false);

    bool                                  isComponentExists(const std::string &name) const override;
    DeviceComponentPtr<IDeviceComponent>  getComponent(const std::string &name, bool throwExIfNotFound = true) override;
    DeviceComponentPtr<IPropertyAccessor> getPropertyAccessor() override;

    // todo: move get sensor functions to base class

protected:
    DeviceComponentLock tryLockResource();

private:
    std::shared_ptr<Context> ctx_;  // handle the lifespan of the context

    std::recursive_timed_mutex                               componentMutex_;
    std::map<std::string, std::shared_ptr<IDeviceComponent>> components_;
    std::map<std::string, std::shared_ptr<IDeviceComponent>> lockRequiredComponents_;
};

}  // namespace libobsensor