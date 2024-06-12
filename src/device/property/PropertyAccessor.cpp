#include "PropertyAccessor.hpp"
#include "exception/ObException.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {

PropertyAccessor::PropertyAccessor() {}

void PropertyAccessor::registerProperty(uint32_t propertyId, OBPermissionType permission, std::shared_ptr<IPropertyPort> port) {
    properties_[propertyId] = { propertyId, permission, port };
}

void PropertyAccessor::aliasProperty(uint32_t aliasId, uint32_t propertyId) {
    auto it = properties_.find(propertyId);
    if(it == properties_.end()) {
        throw invalid_value_exception("Property not found for aliasing");
    }
    properties_[aliasId] = it->second;
}

bool PropertyAccessor::checkProperty(uint32_t propertyId, OBPermissionType permission) const {
    auto it = properties_.find(propertyId);
    if(it == properties_.end()) {
        return false;
    }
    return it->second.permission & permission;
}

void PropertyAccessor::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_WRITE)) {
        throw invalid_value_exception("Property not writable");
    }

    auto  it         = properties_.find(propertyId);
    auto &port       = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId!= propertyId){
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    port->setPropertyValue(propId, value);
    LOG_DEBUG("Property {} set to {}|{}", propId, value.intValue, value.floatValue);
}

void PropertyAccessor::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it         = properties_.find(propertyId);
    auto &port       = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId!= propertyId){
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    port->getPropertyValue(propId, value);
    LOG_DEBUG("Property {} get as {}|{}", propId, value->intValue, value->floatValue);
}

void PropertyAccessor::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it         = properties_.find(propertyId);
    auto &port       = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId!= propertyId){
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    port->getPropertyRange(propId, range);
    LOG_DEBUG("Property {} range as {}-{} step {} def {}|{}-{} step {} def {}", propId, range->min.intValue, range->max.intValue, range->step.intValue,
              range->def.intValue, range->min.floatValue, range->max.floatValue, range->step.floatValue, range->def.floatValue);
}

void PropertyAccessor::setFirmwareData(uint32_t propertyId, const std::vector<uint8_t> &data) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_WRITE)) {
        throw invalid_value_exception("Property not writable");
    }

    auto  it         = properties_.find(propertyId);
    auto &port       = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId!= propertyId){
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port);
    extensionPort->setFirmwareData(propId, data);
    LOG_DEBUG("Property {} set firmware data successfully", propId);
}

const std::vector<uint8_t> &PropertyAccessor::getFirmwareData(uint32_t propertyId) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!checkProperty(propertyId, OB_PERMISSION_READ)) {
        throw invalid_value_exception("Property not readable");
    }

    auto  it         = properties_.find(propertyId);
    auto &port       = it->second.port;
    auto &propId = it->second.propertyId;
    if(propId!= propertyId){
        LOG_DEBUG("Property {} alias to {}", propId, propertyId);
    }

    auto        extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port);
    const auto &data          = extensionPort->getFirmwareData(propId);
    LOG_DEBUG("Property {} get firmware data successfully, size {}", propId, data.size());
    return data;
}
}  // namespace libobsensor