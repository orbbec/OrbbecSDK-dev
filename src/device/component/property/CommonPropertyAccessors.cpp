#include "CommonPropertyAccessors.hpp"

#include "IDeviceMonitor.hpp"
#include "IAlgParamManager.hpp"

#include "libobsensor/h/ObTypes.h"
#include "utils/Utils.hpp"
namespace libobsensor {

DeviceComponentPropertyAccessor::DeviceComponentPropertyAccessor(IDevice *device, DeviceComponentId compId) : device_(device), compId_(compId) {}

void DeviceComponentPropertyAccessor::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    auto propertyAccessor = device_->getComponentT<IBasicPropertyAccessor>(compId_);
    propertyAccessor->setPropertyValue(propertyId, value);
}

void DeviceComponentPropertyAccessor::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    auto propertyAccessor = device_->getComponentT<IBasicPropertyAccessor>(compId_);
    propertyAccessor->getPropertyValue(propertyId, value);
}

void DeviceComponentPropertyAccessor::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    auto propertyAccessor = device_->getComponentT<IBasicPropertyAccessor>(compId_);
    propertyAccessor->getPropertyRange(propertyId, range);
}

FunctionPropertyAccessor::FunctionPropertyAccessor(std::function<OBPropertyValue(uint32_t)> getter, std::function<void(uint32_t, OBPropertyValue)> setter,
                                                   std::function<OBPropertyRange(uint32_t)> rangeGetter)
    : getter_(getter), setter_(setter), rangeGetter_(rangeGetter) {}

void FunctionPropertyAccessor::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    setter_(propertyId, value);
}

void FunctionPropertyAccessor::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    *value = getter_(propertyId);
}

void FunctionPropertyAccessor::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    *range = rangeGetter_(propertyId);
}

LazyPropertyAccessor::LazyPropertyAccessor(std::function<std::shared_ptr<IPropertyAccessor>()> accessorCreator) : accessorCreator_(accessorCreator) {}

void LazyPropertyAccessor::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto basicAccessor = std::dynamic_pointer_cast<IBasicPropertyAccessor>(accessor_);
    basicAccessor->setPropertyValue(propertyId, value);
}

void LazyPropertyAccessor::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto basicAccessor = std::dynamic_pointer_cast<IBasicPropertyAccessor>(accessor_);
    basicAccessor->getPropertyValue(propertyId, value);
}

void LazyPropertyAccessor::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto basicAccessor = std::dynamic_pointer_cast<IBasicPropertyAccessor>(accessor_);
    basicAccessor->getPropertyRange(propertyId, range);
}

LazyExtensionPropertyAccessor::LazyExtensionPropertyAccessor(std::function<std::shared_ptr<IPropertyAccessor>()> accessorCreator)
    : LazyPropertyAccessor(accessorCreator) {}

void LazyExtensionPropertyAccessor::setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IExtensionPropertyAccessor>(accessor_);
    extensionAccessor->setStructureData(propertyId, data);
}

const std::vector<uint8_t> &LazyExtensionPropertyAccessor::getStructureData(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IExtensionPropertyAccessor>(accessor_);
    return extensionAccessor->getStructureData(propertyId);
}

void LazyExtensionPropertyAccessor::getRawData(uint32_t propertyId, GetDataCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IExtensionPropertyAccessor>(accessor_);
    extensionAccessor->getRawData(propertyId, callback);
}

uint16_t LazyExtensionPropertyAccessor::getCmdVersionProtoV1_1(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IExtensionPropertyAccessorV1_1>(accessor_);
    return extensionAccessor->getCmdVersionProtoV1_1(propertyId);
}

const std::vector<uint8_t> &LazyExtensionPropertyAccessor::getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IExtensionPropertyAccessorV1_1>(accessor_);
    return extensionAccessor->getStructureDataProtoV1_1(propertyId, cmdVersion);
}

void LazyExtensionPropertyAccessor::setStructureDataProtoV1_1(uint32_t propertyId, const std::vector<uint8_t> &data, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IExtensionPropertyAccessorV1_1>(accessor_);
    extensionAccessor->setStructureDataProtoV1_1(propertyId, data, cmdVersion);
}

const std::vector<uint8_t> &LazyExtensionPropertyAccessor::getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!accessor_) {
        accessor_ = accessorCreator_();
    }
    auto extensionAccessor = std::dynamic_pointer_cast<IExtensionPropertyAccessorV1_1>(accessor_);
    return extensionAccessor->getStructureDataListProtoV1_1(propertyId, cmdVersion);
}

HeartbeatPropertyAccessor::HeartbeatPropertyAccessor(IDevice *owner) : owner_(owner) {}

void HeartbeatPropertyAccessor::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    utils::unusedVar(propertyId);
    auto deviceMonitor = owner_->getComponentT<IDeviceMonitor>(OB_DEV_COMPONENT_DEVICE_MONITOR);
    if(value.intValue == 1) {
        deviceMonitor->enableHeartbeat();
    }
    else {
        deviceMonitor->disableHeartbeat();
    }
}

void HeartbeatPropertyAccessor::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    utils::unusedVar(propertyId);
    auto deviceMonitor = owner_->getComponentT<IDeviceMonitor>(OB_DEV_COMPONENT_DEVICE_MONITOR);
    value->intValue    = deviceMonitor->isHeartbeatEnabled() ? 1 : 0;
}

void HeartbeatPropertyAccessor::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    OBPropertyValue cur;
    getPropertyValue(propertyId, &cur);
    range->cur           = cur;
    range->min.intValue  = 0;
    range->max.intValue  = 1;
    range->step.intValue = 1;
    range->def.intValue  = 0;
}

BaselinePropertyAccessor::BaselinePropertyAccessor(IDevice *owner) : owner_(owner) {}

void BaselinePropertyAccessor::setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) {
    utils::unusedVar(propertyId);
    utils::unusedVar(data);
    throw unsupported_operation_exception("Baseline params readonly!");
}

const std::vector<uint8_t> &BaselinePropertyAccessor::getStructureData(uint32_t propertyId) {
    utils::unusedVar(propertyId);
    baselineData_.resize(sizeof(OBBaselineCalibrationParam));
    OBBaselineCalibrationParam *param = (OBBaselineCalibrationParam *)baselineData_.data();

    auto algParamManager = owner_->getComponentT<IDisparityAlgParamManager>(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
    auto disparityParam  = algParamManager->getDisparityParam();
    param->baseline      = disparityParam.baseline;
    param->zpd           = static_cast<float>(disparityParam.zpd);
    return baselineData_;
}

void BaselinePropertyAccessor::getRawData(uint32_t propertyId, GetDataCallback callback) {
    utils::unusedVar(propertyId);
    utils::unusedVar(callback);
    throw unsupported_operation_exception("Baseline params readonly!");
}

}  // namespace libobsensor