#include "PropertyAccessor.hpp"
#include "exception/ObException.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {

PropertyAccessor::PropertyAccessor() {}

void PropertyAccessor::registerProperty(uint32_t propertyId, OBPermissionType userPerms, OBPermissionType intPerms, std::shared_ptr<IPropertyPort> port) {
    properties_[propertyId] = { propertyId, userPerms, intPerms, port };
}
void PropertyAccessor::registerProperty(uint32_t propertyId, const std::string &userPermsStr, const std::string &intPermsStr,
                                        std::shared_ptr<IPropertyPort> port) {
    auto             strToPermission = [](const std::string &str) {
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
    extensionPort->setStructureData(propId, data);
    LOG_DEBUG("Property {} set firmware data successfully", propId);
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

    auto        extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port);
    const auto &data          = extensionPort->getStructureData(propId);
    LOG_DEBUG("Property {} get firmware data successfully, size {}", propId, data.size());
    return data;
}
}  // namespace libobsensor