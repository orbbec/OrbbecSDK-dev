#include "DeviceBase.hpp"

#include "context/Context.hpp"
#include "exception/ObException.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {

DeviceBase::DeviceBase() : ctx_(Context::getInstance()) {}

DeviceComponentLock DeviceBase::tryLockResource() {
    DeviceComponentLock resLock(componentMutex_, std::defer_lock);
    if(!resLock.try_lock_for(std::chrono::milliseconds(10000))) {
        throw libobsensor::wrong_api_call_sequence_exception("Resource busy! You can try again later!");
    }
    return resLock;
}

void DeviceBase::registerComponent(const std::string &name, std::shared_ptr<IDeviceComponent> component, bool lockRequired) {
    DeviceComponentLock resLock = tryLockResource();
    auto               &compMap = lockRequired ? lockRequiredComponents_ : components_;

    if(compMap.find(name) != compMap.end()) {
        throw invalid_value_exception(utils::string::to_string() << "Component " << name << " already registered!");
    }
    compMap[name] = component;
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

    auto it = components_.find(name);
    if(it != components_.end()) {
        return true;
    }

    it = lockRequiredComponents_.find(name);
    if(it != lockRequiredComponents_.end()) {
        return true;
    }

    return false;
}

DeviceComponentPtr<IDeviceComponent> DeviceBase::getComponent(const std::string &name, bool throwExIfNotFound) {
    auto sensorTypeIt = componentNameToSensorTypeMap.find(name);
    if(sensorTypeIt != componentNameToSensorTypeMap.end()) {
        auto sensor = getSensor(sensorTypeIt->second);
        return sensor.as<IDeviceComponent>();
    }

    // components dons't require a lock
    auto it = components_.find(name);
    if(it != components_.end()) {
        return DeviceComponentPtr<IDeviceComponent>(it->second);
    }

    // lockRequired components require a lock
    DeviceComponentLock resLock = tryLockResource();
    it                          = lockRequiredComponents_.find(name);
    if(it != lockRequiredComponents_.end()) {
        return DeviceComponentPtr<IDeviceComponent>(it->second, std::move(resLock));
    }

    if(throwExIfNotFound) {
        throw invalid_value_exception(utils::string::to_string() << "Component " << name << " not found!");
    }
    return DeviceComponentPtr<IDeviceComponent>();
}

DeviceComponentPtr<IPropertyAccessor> DeviceBase::getPropertyAccessor() {
    auto comp          = getComponent(OB_DEV_COMPONENT_PROP_ACCESSOR, true);
    auto propProcessor = comp.as<IPropertyAccessor>();
    return std::move(propProcessor);
}

}  // namespace libobsensor