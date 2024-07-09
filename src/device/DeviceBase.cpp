#include "DeviceBase.hpp"

#include "context/Context.hpp"
#include "exception/ObException.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {

DeviceBase::DeviceBase() : ctx_(Context::getInstance()), isDeactivated_(false) {}

DeviceBase::~DeviceBase() noexcept = default;

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

void DeviceBase::registerComponent(const std::string &name, std::function<std::shared_ptr<IDeviceComponent>()> creator, bool lockRequired) {
    DeviceComponentLock resLock = tryLockResource();
    ComponentItem       item;
    item.name         = name;
    item.component    = nullptr;
    item.lockRequired = lockRequired;
    item.initialized  = false;
    item.creator      = creator;
    components_.emplace_back(item);
}

void DeviceBase::registerComponent(const std::string &name, std::shared_ptr<IDeviceComponent> component, bool lockRequired) {
    DeviceComponentLock resLock = tryLockResource();
    ComponentItem       item;
    item.name         = name;
    item.component    = component;
    item.lockRequired = lockRequired;
    item.initialized  = true;
    item.creator      = nullptr;
    components_.emplace_back(item);
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

    do {
        if(it == components_.end()) {
            break;
        }

        if(!it->component && (it->initialized || !it->creator)) {
            break;
        }
        if(!it->component) {
            it->initialized = true;
            it->component   = it->creator();
        }

        if(!it->lockRequired) {
            return DeviceComponentPtr<IDeviceComponent>(it->component);
        }
        DeviceComponentLock resLock = tryLockResource();
        return DeviceComponentPtr<IDeviceComponent>(it->component, std::move(resLock));
    } while(false);

    if(throwExIfNotFound) {
        throw invalid_value_exception(utils::string::to_string() << "Component " << name << " not found!");
    }
    return DeviceComponentPtr<IDeviceComponent>();
}

DeviceComponentPtr<IPropertyAccessor> DeviceBase::getPropertyAccessor() {
    return getComponentT<IPropertyAccessor>(OB_DEV_COMPONENT_PROP_ACCESSOR, true);
}

std::shared_ptr<IFilter> DeviceBase::getSensorFrameFilter(const std::string &name, OBSensorType type, bool throwIfNotFound) {
    auto filterIter =
        std::find_if(sensorFrameFilters_.begin(), sensorFrameFilters_.end(), [name, type](const std::pair<OBSensorType, std::shared_ptr<IFilter>> &pair) {
            if(type == OB_SENSOR_ACCEL || type == OB_SENSOR_GYRO) {
                return (pair.first == OB_SENSOR_ACCEL || pair.first == OB_SENSOR_GYRO) && (pair.second->getName() == name);
            }
            else {
                return (pair.first == type) && (pair.second->getName() == name);
            }
        });

    if(filterIter != sensorFrameFilters_.end()) {
        return filterIter->second;
    }

    auto filterFactory = FilterFactory::getInstance();
    TRY_EXECUTE({
        auto filter               = filterFactory->createFilter(name);
        sensorFrameFilters_[type] = filter;
        return filter;
    });

    if(throwIfNotFound) {
        throw invalid_value_exception(utils::string::to_string() << "Filter " << name << " not found!");
    }

    return nullptr;
}

}  // namespace libobsensor