#pragma once
#include "IProperty.hpp"
#include "IDevice.hpp"

namespace libobsensor {
class G330PropertyAccessor : public IPropertyAccessor {
public:
    explicit G330PropertyAccessor(IDevice *owner);
    virtual ~G330PropertyAccessor() noexcept = default;

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