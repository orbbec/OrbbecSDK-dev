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

};  // namespace libobsensor