#pragma once

#include "IProperty.hpp"
#include <map>

namespace libobsensor {

class PropertyAccessor : public IPropertyAccessor {
    struct PropertyItem {
        uint32_t                       propertyId;
        OBPermissionType               permission;
        std::shared_ptr<IPropertyPort> port;
    };

public:
    PropertyAccessor();
    ~PropertyAccessor() noexcept = default;

    void registerProperty(uint32_t propertyId, OBPermissionType permission, std::shared_ptr<IPropertyPort> port) override;
    void aliasProperty(uint32_t aliasId, uint32_t propertyId) override;

    bool checkProperty(uint32_t propertyId, OBPermissionType permission) const override;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

    void                        setFirmwareData(uint32_t propertyId, const std::vector<uint8_t> &data) override;
    const std::vector<uint8_t> &getFirmwareData(uint32_t propertyId) override;

private:
    std::mutex mutex_;
    std::map<uint32_t, PropertyItem> properties_;
};

}  // namespace libobsensor