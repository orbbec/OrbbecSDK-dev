#pragma once

#include "IDevice.hpp"

#include <memory>
#include <map>

namespace libobsensor {

class Context;

class DeviceBase : public IDevice {
private:
    struct ComponentItem {
        std::string                       name;
        std::shared_ptr<IDeviceComponent> component;
        bool                              lockRequired;
    };

public:
    DeviceBase();
    virtual ~DeviceBase() noexcept;

    void registerComponent(const std::string &name, std::shared_ptr<IDeviceComponent> component, bool lockRequired = false);

    bool                                  isComponentExists(const std::string &name) const override;
    DeviceComponentPtr<IDeviceComponent>  getComponent(const std::string &name, bool throwExIfNotFound = true) override;
    DeviceComponentPtr<IPropertyAccessor> getPropertyAccessor() override;

    void deactivate() override;

    // todo: move get sensor functions to base class

protected:
    void                clearComponents();
    DeviceComponentLock tryLockResource();

private:
    std::shared_ptr<Context> ctx_;  // handle the lifespan of the context

    std::recursive_timed_mutex                               componentMutex_;
    std::vector<ComponentItem>                               components_;  // using vector control destroy order of components

    std::atomic<bool> isDeactivated_;
};

}  // namespace libobsensor