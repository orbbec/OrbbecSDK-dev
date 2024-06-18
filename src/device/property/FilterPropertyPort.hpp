#pragma once

#include "IProperty.hpp"
#include "IFilter.hpp"

namespace libobsensor {
class FilterPropertyPort : public IPropertyPort {
public:
    FilterPropertyPort(const std::shared_ptr<IFilter> &backend);
    ~FilterPropertyPort() noexcept override = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    std::shared_ptr<IFilter> backend_;
};
}  // namespace libobsensor