#pragma once

#include "IProperty.hpp"
#include "InternalProperty.hpp"
#include "openobsdk/h/Property.h"
#include "PropertyHelper.hpp"
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
    void setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data, PropertyAccessType accessType) override;
    const std::vector<uint8_t> &getStructureData(uint32_t propertyId, PropertyAccessType accessType) override;

    void getRawData(uint32_t propertyId, GetDataCallback callback, PropertyAccessType accessType = PROP_ACCESS_INTERNAL) override;

    uint16_t                    getCmdVersionProtoV1_1(uint32_t propertyId, PropertyAccessType accessType = PROP_ACCESS_INTERNAL) override;
    const std::vector<uint8_t> &getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion,
                                                          PropertyAccessType accessType = PROP_ACCESS_INTERNAL) override;
    const std::vector<uint8_t> &getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion,
                                                              PropertyAccessType accessType = PROP_ACCESS_INTERNAL) override;

    OBPropertyItem getSupportedPropertyItem(uint32_t index) override;

    std::vector<OBPropertyItem> getProperties() const;
    int                         getSupportedPropertyCount() override;
    bool isPropertySupported(uint32_t propertId, OBPermissionType permissionType, PropertyAccessType accessType) override;

private:
    std::mutex                       mutex_;
    std::map<uint32_t, PropertyItem> properties_;
    std::vector<OBPropertyItem>      propertiesVec_;

private:
    void appendToPropertyMap(uint32_t propertyId, OBPermissionType userPerms, OBPermissionType intPerms);
};

}  // namespace libobsensor