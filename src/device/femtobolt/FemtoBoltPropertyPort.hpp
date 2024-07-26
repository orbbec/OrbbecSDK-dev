#pragma once
#include "IProperty.hpp"
#include "IDevice.hpp"

namespace libobsensor {
class FemtoBoltPropertyPort : public IPropertyPort {
public:
    explicit FemtoBoltPropertyPort(IDevice *owner);
    virtual ~FemtoBoltPropertyPort() noexcept = default;

    virtual void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    virtual void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    virtual void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    void markOutputDisparityFrame(bool enable);

private:
    IDevice *owner_;
};
}  // namespace libobsensor