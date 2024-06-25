#include "PropertyAccessor.hpp"
#include "exception/ObException.hpp"
#include "logger/Logger.hpp"
#include "utils/Utils.hpp"
#include <memory>

namespace libobsensor {

PropertyAccessor::PropertyAccessor() {}

void PropertyAccessor::registerProperty(uint32_t propertyId, OBPermissionType userPerms, OBPermissionType intPerms, std::shared_ptr<IPropertyPort> port) {
    properties_[propertyId] = { propertyId, userPerms, intPerms, port };

    appendToPropertyMap(propertyId, userPerms, intPerms);
}

void PropertyAccessor::registerProperty(uint32_t propertyId, const std::string &userPermsStr, const std::string &intPermsStr,
                                        std::shared_ptr<IPropertyPort> port) {
    auto strToPermission = [](const std::string &str) {
        if(str == "r") {
            return OB_PERMISSION_READ;
        }
        else if(str == "w") {
            return OB_PERMISSION_WRITE;
        }
        else if(str == "rw") {
            return OB_PERMISSION_READ_WRITE;
        }
        else {
            return OB_PERMISSION_DENY;
        }
    };
    auto userPerms = strToPermission(userPermsStr);
    auto intPerms  = strToPermission(intPermsStr);
    registerProperty(propertyId, userPerms, intPerms, port);
}

void addProperty(std::vector<OBPropertyItem> &vec, int propertyId, const char *propName, OBPropertyType propType, OBPermissionType perms) {
    OBPropertyItem propertyItem;
    propertyItem.id         = static_cast<OBPropertyID>(propertyId);
    propertyItem.name       = propName;
    propertyItem.type       = propType;
    propertyItem.permission = perms;
    vec.push_back(propertyItem);
}

void PropertyAccessor::appendToPropertyMap(uint32_t propertyId, OBPermissionType userPerms, OBPermissionType intPerms) {
    for(auto &item: OBPropertyBaseInfoMap) {
        auto infoIter = OBPropertyBaseInfoMap.find(item.first);
        if(infoIter == OBPropertyBaseInfoMap.end()) {
            std::string msg = ", id=";
            msg += std::to_string(item.first);
            throw not_implemented_exception(msg);
        }
        if(propertyId == item.first) {
            if(userPerms & 0x3) {
                addProperty(userPropertiesVec_, propertyId, infoIter->second.name, infoIter->second.type, userPerms);
            }

            if(intPerms & 0x3) {
                addProperty(innerPropertiesVec_, propertyId, infoIter->second.name, infoIter->second.type, intPerms);
            }
            break;
        }
    }
}

void PropertyAccessor::aliasProperty(uint32_t aliasId, uint32_t propertyId) {
    auto it = properties_.find(propertyId);
    if(it == properties_.end()) {
        throw invalid_value_exception("Property not found for aliasing");
    }
    properties_[aliasId] = it->second;
}

bool PropertyAccessor::checkProperty(uint32_t propertyId, OBPermissionType permission, PropertyAccessType accessType) const {
    auto it = properties_.find(propertyId);
    if(it == properties_.end()) {
        return false;
    }

    if(accessType == PROP_ACCESS_USER) {
        return it->second.userPermission & permission || permission == OB_PERMISSION_ANY;
    }
    else if(accessType == PROP_ACCESS_INTERNAL) {
        return it->second.InternalPermission & permission || permission == OB_PERMISSION_ANY;
    }

    return false;
}

void PropertyAccessor::setPropertyValue(uint32_t propertyId, OBPropertyValue value, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_WRITE, accessType)) {
        throw invalid_value_exception("Property not writable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    port->setPropertyValue(propId, value);
    LOG_DEBUG("Property {} set to {}|{}", propId, value.intValue, value.floatValue);
}

void PropertyAccessor::getPropertyValue(uint32_t propertyId, OBPropertyValue *value, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ, accessType)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    port->getPropertyValue(propId, value);
    LOG_DEBUG("Property {} get as {}|{}", propId, value->intValue, value->floatValue);
}

// std::vector<OBPropertyItem> PropertyAccessor::getProperties(PropertyAccessType accessType) const{
//     std::unique_lock<std::mutex> lock(mutex_);
//     return properties_;
// }

void PropertyAccessor::getPropertyRange(uint32_t propertyId, OBPropertyRange *range, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ, accessType)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    port->getPropertyRange(propId, range);
    LOG_DEBUG("Property {} range as {}-{} step {} def {}|{}-{} step {} def {}", propId, range->min.intValue, range->max.intValue, range->step.intValue,
              range->def.intValue, range->min.floatValue, range->max.floatValue, range->step.floatValue, range->def.floatValue);
}

void PropertyAccessor::setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_WRITE, accessType)) {
        throw invalid_value_exception("Property not writable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port);
    if(extensionPort == nullptr) {
        throw invalid_value_exception(utils::to_string() << "Property" << propId << " does not support structure data setting");
    }
    extensionPort->setStructureData(propId, data);
    LOG_DEBUG("Property {} set structure data successfully", propId);
}

const std::vector<uint8_t> &PropertyAccessor::getStructureData(uint32_t propertyId, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ, accessType)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port);
    if(extensionPort == nullptr) {
        throw invalid_value_exception(utils::to_string() << "Property" << propId << " does not support structure data getting");
    }
    const auto &data = extensionPort->getStructureData(propId);
    LOG_DEBUG("Property {} get structure data successfully, size {}", propId, data.size());
    return data;
}

void PropertyAccessor::getRawData(uint32_t propertyId, GetDataCallback callback, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ, accessType)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port);
    if(extensionPort == nullptr) {
        throw invalid_value_exception(utils::to_string() << "Property" << propId << " does not support raw data getting");
    }

    extensionPort->getRawData(propId, callback);  // todo: add async support

    LOG_DEBUG("Property {} get raw data successfully", propId);
}

uint16_t PropertyAccessor::getCmdVersionProtoV1_1(uint32_t propertyId, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ, accessType)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPortV1_1>(port);
    if(extensionPort == nullptr) {
        throw invalid_value_exception(utils::to_string() << "Property" << propId << " does not support cmd version getting");
    }

    auto ver = extensionPort->getCmdVersionProtoV1_1(propId);
    LOG_DEBUG("Property {} get cmd version successfully, version {}", propId, ver);
    return ver;
}

const std::vector<uint8_t> &PropertyAccessor::getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ, accessType)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPortV1_1>(port);
    if(extensionPort == nullptr) {
        throw invalid_value_exception(utils::to_string() << "Property" << propId << " does not support structure data getting over proto v1.1");
    }
    const auto &data = extensionPort->getStructureDataProtoV1_1(propId, cmdVersion);
    LOG_DEBUG("Property {} get structure data successfully over proto v1.1, size {}", propId, data.size());
    return data;
}

const std::vector<uint8_t> &PropertyAccessor::getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion, PropertyAccessType accessType) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ, accessType)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it     = properties_.find(propertyId);
    auto &port   = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId != propertyId) {
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPortV1_1>(port);
    if(extensionPort == nullptr) {
        throw invalid_value_exception(utils::to_string() << "Property" << propId << " does not support structure data list getting over proto v1.1");
    }
    const auto &dataList = extensionPort->getStructureDataListProtoV1_1(propId, cmdVersion);
    LOG_DEBUG("Property {} get structure data list successfully over proto v1.1, size {}", propId, dataList.size());
    return dataList;
}

const std::vector<OBPropertyItem> &PropertyAccessor::getAvailableProperties(PropertyAccessType accessType) {
    if(accessType == PROP_ACCESS_USER) {
        return userPropertiesVec_;
    }
    else if(accessType == PROP_ACCESS_INTERNAL) {
        return innerPropertiesVec_;
    }

    static const std::vector<OBPropertyItem> emptyVec;
    return emptyVec;
}

}  // namespace libobsensor