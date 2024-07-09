#pragma once

#include "IDevice.hpp"
#include "IProperty.hpp"

namespace libobsensor {
class DeviceComponentPropertyPortWrapper : public IPropertyPort {
public:
    DeviceComponentPropertyPortWrapper(IDevice *device, const std::string &compName);
    virtual ~DeviceComponentPropertyPortWrapper() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    IDevice    *device_;
    std::string compName_;
};

class FunctionPropertyPortWrapper : public IPropertyPort {
public:
    FunctionPropertyPortWrapper(std::function<OBPropertyValue(uint32_t)> getter, std::function<void(uint32_t, OBPropertyValue)> setter,
                                std::function<OBPropertyRange(uint32_t)> rangeGetter);

    virtual ~FunctionPropertyPortWrapper() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    std::function<OBPropertyValue(uint32_t)>       getter_;
    std::function<void(uint32_t, OBPropertyValue)> setter_;
    std::function<OBPropertyRange(uint32_t)>       rangeGetter_;
};
}  // namespace libobsensor