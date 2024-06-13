#pragma once

#include "IProperty.hpp"
#include "InternalProperty.hpp"
#include "openobsdk/h/Property.h"
#include <map>

namespace libobsensor {

class PropertyAccessor : public IPropertyAccessor {
    struct PropertyItem {
        uint32_t                       propertyId;
        OBPermissionType               userPermission;
        OBPermissionType               InternalPermission;
        std::shared_ptr<IPropertyPort> port;
    };

public:
    PropertyAccessor();
    ~PropertyAccessor() noexcept = default;

    void registerProperty(uint32_t propertyId, OBPermissionType userPerms, OBPermissionType intPerms, std::shared_ptr<IPropertyPort> port) override;
    void registerProperty(uint32_t propertyId, const std::string &userPermsStr, const std::string &intPermsStr, std::shared_ptr<IPropertyPort> port) override;
    void aliasProperty(uint32_t aliasId, uint32_t propertyId) override;

    bool checkProperty(uint32_t propertyId, OBPermissionType permission, PropertyAccessType accessType) const override;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value, PropertyAccessType accessType) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value, PropertyAccessType accessType) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range, PropertyAccessType accessType) override;

    void                        setFirmwareData(uint32_t propertyId, const std::vector<uint8_t> &data, PropertyAccessType accessType) override;
    const std::vector<uint8_t> &getFirmwareData(uint32_t propertyId, PropertyAccessType accessType) override;

private:
    std::mutex                       mutex_;
    std::map<uint32_t, PropertyItem> properties_;
};

}  // namespace libobsensor