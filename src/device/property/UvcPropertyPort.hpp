#pragma once
#include "IProperty.hpp"
#include "ISourcePort.hpp"

namespace libobsensor {

class UvcPropertyPort: public IPropertyPort {
public:
    UvcPropertyPort(const std::shared_ptr<ISourcePort>& backend);
    ~UvcPropertyPort() noexcept override = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    std::shared_ptr<ISourcePort> backend_;
};

}  // namespace libobsensor
