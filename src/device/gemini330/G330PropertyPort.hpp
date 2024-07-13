#pragma once
#include "IProperty.hpp"
#include "IDevice.hpp"

namespace libobsensor {
class G330PropertyPort : public IPropertyPort{
public:
    explicit G330PropertyPort(IDevice *owner);
    virtual ~G330PropertyPort() noexcept = default;

    virtual void setPropertyValue(uint32_t propertyId, OBPropertyValue value)   override;
    virtual void getPropertyValue(uint32_t propertyId, OBPropertyValue *value)  override;
    virtual void getPropertyRange(uint32_t propertyId, OBPropertyRange *range)  override;

private:
    void markOutputDisparityFrame(bool enable);

private:
    IDevice *owner_;

    bool hwDisparityToDepthEnabled_;
    bool swDisparityToDepthEnabled_;
};
}