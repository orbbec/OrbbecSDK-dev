#include "CommonPropertyAccessors.hpp"

namespace libobsensor {

DeviceComponentPropertyAccessorWrapper::DeviceComponentPropertyAccessorWrapper(IDevice *device, const std::string &compName)
    : device_(device), compName_(compName) {}

void DeviceComponentPropertyAccessorWrapper::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    auto comp             = device_->getComponent(compName_);
    auto propertyAccessor = comp.as<IPropertyAccessor>();
    propertyAccessor->setPropertyValue(propertyId, value);
}

void DeviceComponentPropertyAccessorWrapper::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    auto comp             = device_->getComponent(compName_);
    auto propertyAccessor = comp.as<IPropertyAccessor>();
    propertyAccessor->getPropertyValue(propertyId, value);
}

void DeviceComponentPropertyAccessorWrapper::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    auto comp             = device_->getComponent(compName_);
    auto propertyAccessor = comp.as<IPropertyAccessor>();
    propertyAccessor->getPropertyRange(propertyId, range);
}

FunctionPropertyAccessorWrapper::FunctionPropertyAccessorWrapper(std::function<OBPropertyValue(uint32_t)>       getter,
                                                                 std::function<void(uint32_t, OBPropertyValue)> setter,
                                                                 std::function<OBPropertyRange(uint32_t)>       rangeGetter)
    : getter_(getter), setter_(setter), rangeGetter_(rangeGetter) {}

void FunctionPropertyAccessorWrapper::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    setter_(propertyId, value);
}

void FunctionPropertyAccessorWrapper::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    *value = getter_(propertyId);
}

void FunctionPropertyAccessorWrapper::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    *range = rangeGetter_(propertyId);
}

LazyPropertyAccessorWrapper::LazyPropertyAccessorWrapper(std::function<std::shared_ptr<IPropertyAccessor>()> accessorCreator)
    : accessorCreator_(accessorCreator) {}

void LazyPropertyAccessorWrapper::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    accessor_->setPropertyValue(propertyId, value);
}

void LazyPropertyAccessorWrapper::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    accessor_->getPropertyValue(propertyId, value);
}

void LazyPropertyAccessorWrapper::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    accessor_->getPropertyRange(propertyId, range);
}

LazyPropertyExtensionAccessorWrapper::LazyPropertyExtensionAccessorWrapper(std::function<std::shared_ptr<IPropertyExtensionAccessor>()> accessorCreator)
    : LazyPropertyAccessorWrapper(accessorCreator) {}

void LazyPropertyExtensionAccessorWrapper::setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IPropertyExtensionAccessor>(accessor_);
    extensionAccessor->setStructureData(propertyId, data);
}

const std::vector<uint8_t> &LazyPropertyExtensionAccessorWrapper::getStructureData(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IPropertyExtensionAccessor>(accessor_);
    return extensionAccessor->getStructureData(propertyId);
}

void LazyPropertyExtensionAccessorWrapper::getRawData(uint32_t propertyId, GetDataCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IPropertyExtensionAccessor>(accessor_);
    extensionAccessor->getRawData(propertyId, callback);
}

uint16_t LazyPropertyExtensionAccessorWrapper::getCmdVersionProtoV1_1(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IPropertyExtensionAccessorV1_1>(accessor_);
    return extensionAccessor->getCmdVersionProtoV1_1(propertyId);
}

const std::vector<uint8_t> &LazyPropertyExtensionAccessorWrapper::getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IPropertyExtensionAccessorV1_1>(accessor_);
    return extensionAccessor->getStructureDataProtoV1_1(propertyId, cmdVersion);
}

void LazyPropertyExtensionAccessorWrapper::setStructureDataProtoV1_1(uint32_t propertyId, const std::vector<uint8_t> &data, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IPropertyExtensionAccessorV1_1>(accessor_);
    extensionAccessor->setStructureDataProtoV1_1(propertyId, data, cmdVersion);
}

const std::vector<uint8_t> &LazyPropertyExtensionAccessorWrapper::getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IPropertyExtensionAccessorV1_1>(accessor_);
    return extensionAccessor->getStructureDataListProtoV1_1(propertyId, cmdVersion);
}

}  // namespace libobsensor