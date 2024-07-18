#include "CommonPropertyPorts.hpp"

namespace libobsensor {

DeviceComponentPropertyPortWrapper::DeviceComponentPropertyPortWrapper(IDevice *device, const std::string &compName) : device_(device), compName_(compName) {}

void DeviceComponentPropertyPortWrapper::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    auto comp         = device_->getComponent(compName_);
    auto propertyPort = comp.as<IPropertyPort>();
    propertyPort->setPropertyValue(propertyId, value);
}

void DeviceComponentPropertyPortWrapper::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    auto comp         = device_->getComponent(compName_);
    auto propertyPort = comp.as<IPropertyPort>();
    propertyPort->getPropertyValue(propertyId, value);
}

void DeviceComponentPropertyPortWrapper::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    auto comp         = device_->getComponent(compName_);
    auto propertyPort = comp.as<IPropertyPort>();
    propertyPort->getPropertyRange(propertyId, range);
}

FunctionPropertyPortWrapper::FunctionPropertyPortWrapper(std::function<OBPropertyValue(uint32_t)> getter, std::function<void(uint32_t, OBPropertyValue)> setter,
                                                         std::function<OBPropertyRange(uint32_t)> rangeGetter)
    : getter_(getter), setter_(setter), rangeGetter_(rangeGetter) {}

void FunctionPropertyPortWrapper::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    setter_(propertyId, value);
}

void FunctionPropertyPortWrapper::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    *value = getter_(propertyId);
}

void FunctionPropertyPortWrapper::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    *range = rangeGetter_(propertyId);
}

LazyPropertyPortWrapper::LazyPropertyPortWrapper(std::function<std::shared_ptr<IPropertyPort>()> portCreator) : portCreator_(portCreator) {}

void LazyPropertyPortWrapper::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    port_->setPropertyValue(propertyId, value);
}

void LazyPropertyPortWrapper::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    port_->getPropertyValue(propertyId, value);
}

void LazyPropertyPortWrapper::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    port_->getPropertyRange(propertyId, range);
}

LazyPropertyExtensionPortWrapper::LazyPropertyExtensionPortWrapper(std::function<std::shared_ptr<IPropertyExtensionPort>()> portCreator)
    : LazyPropertyPortWrapper(portCreator) {}

void LazyPropertyExtensionPortWrapper::setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port_);
    extensionPort->setStructureData(propertyId, data);
}

const std::vector<uint8_t> &LazyPropertyExtensionPortWrapper::getStructureData(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port_);
    return extensionPort->getStructureData(propertyId);
}

void LazyPropertyExtensionPortWrapper::getRawData(uint32_t propertyId, GetDataCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPort>(port_);
    extensionPort->getRawData(propertyId, callback);
}

uint16_t LazyPropertyExtensionPortWrapper::getCmdVersionProtoV1_1(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPortV1_1>(port_);
    return extensionPort->getCmdVersionProtoV1_1(propertyId);
}

const std::vector<uint8_t> &LazyPropertyExtensionPortWrapper::getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPortV1_1>(port_);
    return extensionPort->getStructureDataProtoV1_1(propertyId, cmdVersion);
}

void LazyPropertyExtensionPortWrapper::setStructureDataProtoV1_1(uint32_t propertyId, const std::vector<uint8_t> &data, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPortV1_1>(port_);
    extensionPort->setStructureDataProtoV1_1(propertyId, data, cmdVersion);
}

const std::vector<uint8_t> &LazyPropertyExtensionPortWrapper::getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!port_) {
        port_ = portCreator_();
    }
    auto extensionPort = std::dynamic_pointer_cast<IPropertyExtensionPortV1_1>(port_);
    return extensionPort->getStructureDataListProtoV1_1(propertyId, cmdVersion);
}

}  // namespace libobsensor