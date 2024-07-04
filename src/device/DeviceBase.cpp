#include "DeviceBase.hpp"

#include "context/Context.hpp"
#include "exception/ObException.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {

DeviceBase::DeviceBase() : ctx_(Context::getInstance()), isDeactivated_(false) {}

DeviceBase::~DeviceBase() {}

void DeviceBase::deactivate() {
    clearComponents();
    isDeactivated_ = true;
}

DeviceComponentLock DeviceBase::tryLockResource() {
    if(isDeactivated_) {
        throw libobsensor::wrong_api_call_sequence_exception("Device is deactivated/disconnected!");
    }
    DeviceComponentLock resLock(componentMutex_, std::defer_lock);
    if(!resLock.try_lock_for(std::chrono::milliseconds(10000))) {
        throw libobsensor::wrong_api_call_sequence_exception("Resource busy! You can try again later!");
    }
    return resLock;
}

void DeviceBase::registerComponent(const std::string &name, std::shared_ptr<IDeviceComponent> component, bool lockRequired) {
    DeviceComponentLock resLock = tryLockResource();
    components_.push_back({ name, component, lockRequired });
}

void DeviceBase::clearComponents() {
    components_.clear();
}

const std::map<std::string, OBSensorType> componentNameToSensorTypeMap = {
    { OB_DEV_COMPONENT_COLOR_SENSOR, OB_SENSOR_COLOR },
    { OB_DEV_COMPONENT_DEPTH_SENSOR, OB_SENSOR_DEPTH },
    { OB_DEV_COMPONENT_IR_SENSOR, OB_SENSOR_IR },
    { OB_DEV_COMPONENT_LEFT_IR_SENSOR, OB_SENSOR_IR_LEFT },
    { OB_DEV_COMPONENT_RIGHT_IR_SENSOR, OB_SENSOR_IR_RIGHT },
    { OB_DEV_COMPONENT_GYRO_SENSOR, OB_SENSOR_GYRO },
    { OB_DEV_COMPONENT_ACCEL_SENSOR, OB_SENSOR_ACCEL },
};

bool DeviceBase::isComponentExists(const std::string &name) const {
    auto sensorTypeIt = componentNameToSensorTypeMap.find(name);
    if(sensorTypeIt != componentNameToSensorTypeMap.end()) {
        auto sensorTypeList = getSensorTypeList();
        auto iter =
            std::find_if(sensorTypeList.begin(), sensorTypeList.end(), [sensorTypeIt](const OBSensorType &type) { return type == sensorTypeIt->second; });
        if(iter != sensorTypeList.end()) {
            return true;
        }
    }

    auto it = std::find_if(components_.begin(), components_.end(), [name](const ComponentItem &item) { return item.name == name; });
    if(it != components_.end()) {
        return true;
    }

    return false;
}

DeviceComponentPtr<IDeviceComponent> DeviceBase::getComponent(const std::string &name, bool throwExIfNotFound) {
    if(isDeactivated_) {
        throw libobsensor::wrong_api_call_sequence_exception("Device is deactivated/disconnected!");
    }

    auto sensorTypeIt = componentNameToSensorTypeMap.find(name);
    if(sensorTypeIt != componentNameToSensorTypeMap.end()) {
        auto sensor = getSensor(sensorTypeIt->second);
        return sensor.as<IDeviceComponent>();
    }

    auto it = std::find_if(components_.begin(), components_.end(), [name](const ComponentItem &item) { return item.name == name; });
    if(it != components_.end()) {
        if(!it->lockRequired) {
            return DeviceComponentPtr<IDeviceComponent>(it->component);
        }
        DeviceComponentLock resLock = tryLockResource();
        return DeviceComponentPtr<IDeviceComponent>(it->component, std::move(resLock));
    }

    if(throwExIfNotFound) {
        throw invalid_value_exception(utils::string::to_string() << "Component " << name << " not found!");
    }
    return DeviceComponentPtr<IDeviceComponent>();
}

DeviceComponentPtr<IPropertyAccessor> DeviceBase::getPropertyAccessor() {
    return getComponentT<IPropertyAccessor>(OB_DEV_COMPONENT_PROP_ACCESSOR, true);
}

}  // namespace libobsensor