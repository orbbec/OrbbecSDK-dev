#include "DeviceBase.hpp"

#include "context/Context.hpp"
#include "exception/ObException.hpp"
#include "utils/Utils.hpp"
#include "property/InternalProperty.hpp"

namespace libobsensor {

DeviceBase::DeviceBase(const std::shared_ptr<const IDeviceEnumInfo> &info) : enumInfo_(info), ctx_(Context::getInstance()), isDeactivated_(false) {
    extensionInfo_["AllSensorsUsingSameClock"] = "true";
}

std::shared_ptr<const DeviceInfo> DeviceBase::getInfo() const {
    return deviceInfo_;
}

const std::string &DeviceBase::getExtensionInfo(const std::string &infoKey) const {
    auto it = extensionInfo_.find(infoKey);
    if(it != extensionInfo_.end()) {
        return it->second;
    }
    throw invalid_value_exception(utils::string::to_string() << "Extension info " << infoKey << " not found!");
}

DeviceBase::~DeviceBase() noexcept = default;

void DeviceBase::deactivate() {
    std::lock_guard<std::recursive_mutex> lock(componentsMutex_);
    components_.clear();
    sensorPortInfos_.clear();
    isDeactivated_ = true;
}

void DeviceBase::reboot() {
    auto propServer = getPropertyServer();
    if(propServer->isPropertySupported(OB_PROP_DEVICE_RESET_BOOL, PROP_OP_WRITE, PROP_ACCESS_INTERNAL)) {
        propServer->setPropertyValueT(OB_PROP_DEVICE_RESET_BOOL, true);
    }
    else if(propServer->isPropertySupported(OB_PROP_REBOOT_DEVICE_BOOL, PROP_OP_WRITE, PROP_ACCESS_INTERNAL)) {
        propServer->setPropertyValueT(OB_PROP_REBOOT_DEVICE_BOOL, true);
    }
    else {
        throw invalid_value_exception("Device does not support reboot!");
    }
    deactivate();
}

DeviceComponentLock DeviceBase::tryLockResource() {
    if(isDeactivated_) {
        throw libobsensor::wrong_api_call_sequence_exception("Device is deactivated/disconnected!");
    }
    DeviceComponentLock resLock(resourceMutex_, std::defer_lock);
    if(!resLock.try_lock_for(std::chrono::milliseconds(10000))) {
        throw libobsensor::wrong_api_call_sequence_exception("Resource busy! You can try again later!");
    }
    return resLock;
}

void DeviceBase::registerComponent(DeviceComponentId compId, std::function<std::shared_ptr<IDeviceComponent>()> creator, bool lockRequired) {
    std::lock_guard<std::recursive_mutex> lock(componentsMutex_);
    ComponentItem                         item;
    item.compId       = compId;
    item.component    = nullptr;
    item.lockRequired = lockRequired;
    item.initialized  = false;
    item.creator      = creator;
    components_.emplace_back(item);
}

void DeviceBase::registerComponent(DeviceComponentId compId, std::shared_ptr<IDeviceComponent> component, bool lockRequired) {
    std::lock_guard<std::recursive_mutex> lock(componentsMutex_);
    ComponentItem                         item;
    item.compId       = compId;
    item.component    = component;
    item.lockRequired = lockRequired;
    item.initialized  = true;
    item.creator      = nullptr;
    components_.emplace_back(item);
}

bool DeviceBase::isComponentExists(DeviceComponentId compId) const {
    std::lock_guard<std::recursive_mutex> lock(componentsMutex_);
    auto it = std::find_if(components_.begin(), components_.end(), [compId](const ComponentItem &item) { return item.compId == compId; });
    if(it != components_.end()) {
        return true;
    }

    return false;
}

bool DeviceBase::isComponentCreated(DeviceComponentId compId) const {
    std::lock_guard<std::recursive_mutex> lock(componentsMutex_);
    auto it = std::find_if(components_.begin(), components_.end(), [compId](const ComponentItem &item) { return item.compId == compId; });

    if(it != components_.end()) {
        return it->component != nullptr;
    }

    return false;
}

DeviceComponentPtr<IDeviceComponent> DeviceBase::getComponent(DeviceComponentId compId, bool throwExIfNotFound) {
    if(isDeactivated_) {
        throw libobsensor::wrong_api_call_sequence_exception("Device is deactivated/disconnected!");
    }

    ComponentItem item = { OB_DEV_COMPONENT_UNKNOWN };
    {
        std::lock_guard<std::recursive_mutex> lock(componentsMutex_);
        auto it = std::find_if(components_.begin(), components_.end(), [compId](const ComponentItem &item) { return item.compId == compId; });

        do {
            if(it == components_.end()) {
                break;
            }

            if(!it->component && (it->initialized || !it->creator)) {
                break;
            }

            if(!it->component) {
                it->initialized = true;
                BEGIN_TRY_EXECUTE({ it->component = it->creator(); })
                CATCH_EXCEPTION_AND_EXECUTE({
                    if(throwExIfNotFound) {
                        throw;
                    }
                    return DeviceComponentPtr<IDeviceComponent>();
                })
            }
            item = *it;

        } while(false);
    }

    if(item.compId != OB_DEV_COMPONENT_UNKNOWN) {
        if(item.lockRequired) {
            DeviceComponentLock resLock = tryLockResource();
            return DeviceComponentPtr<IDeviceComponent>(item.component, std::move(resLock));
        }
        return DeviceComponentPtr<IDeviceComponent>(item.component);
    }

    if(throwExIfNotFound) {
        throw invalid_value_exception(utils::string::to_string() << "Component " << compId << " not found!");
    }
    return DeviceComponentPtr<IDeviceComponent>();
}

DeviceComponentPtr<IPropertyServer> DeviceBase::getPropertyServer() {
    return getComponentT<IPropertyServer>(OB_DEV_COMPONENT_PROPERTY_SERVER, true);
}

void DeviceBase::registerSensorPortInfo(OBSensorType type, std::shared_ptr<const SourcePortInfo> sourcePortInfo) {
    sensorPortInfos_[type] = sourcePortInfo;
}

const std::shared_ptr<const SourcePortInfo> &DeviceBase::getSensorPortInfo(OBSensorType type) const {
    auto it = sensorPortInfos_.find(type);
    if(it != sensorPortInfos_.end()) {
        return it->second;
    }
    throw invalid_value_exception(utils::string::to_string() << "Sensor type " << type << " not found!");
}

bool DeviceBase::isSensorExists(OBSensorType type) const {
    auto it = sensorPortInfos_.find(type);
    if(it != sensorPortInfos_.end()) {
        return true;
    }
    return false;
}

const std::map<OBSensorType, DeviceComponentId> SensorTypeToComponentIdMap = {
    { OB_SENSOR_COLOR, OB_DEV_COMPONENT_COLOR_SENSOR },
    { OB_SENSOR_DEPTH, OB_DEV_COMPONENT_DEPTH_SENSOR },
    { OB_SENSOR_IR, OB_DEV_COMPONENT_IR_SENSOR },
    { OB_SENSOR_IR_LEFT, OB_DEV_COMPONENT_LEFT_IR_SENSOR },
    { OB_SENSOR_IR_RIGHT, OB_DEV_COMPONENT_RIGHT_IR_SENSOR },
    { OB_SENSOR_GYRO, OB_DEV_COMPONENT_GYRO_SENSOR },
    { OB_SENSOR_ACCEL, OB_DEV_COMPONENT_ACCEL_SENSOR },
};

bool DeviceBase::isSensorCreated(OBSensorType type) const {
    auto compId = SensorTypeToComponentIdMap.at(type);
    return isComponentCreated(compId);
}

DeviceComponentPtr<ISensor> DeviceBase::getSensor(OBSensorType type) {
    if(type == OB_SENSOR_IR && !isSensorExists(OB_SENSOR_IR) && isSensorExists(OB_SENSOR_IR_LEFT)) {
        type = OB_SENSOR_IR_LEFT;
        LOG_WARN("For stereo camera, dose not support IR sensor, use IR left instead here.");
    }
    auto compId = SensorTypeToComponentIdMap.at(type);
    return getComponentT<ISensor>(compId, true);
}

std::vector<OBSensorType> DeviceBase::getSensorTypeList() const {
    std::vector<OBSensorType> sensorTypeList;
    for(auto &item: sensorPortInfos_) {
        sensorTypeList.push_back(item.first);
    }
    return sensorTypeList;
}

bool DeviceBase::hasAnySensorStreamActivated() {
    for(auto &item: sensorPortInfos_) {
        if(isSensorCreated(item.first)) {
            auto sensor = getSensor(item.first);
            if(sensor && sensor->isStreamActivated()) {
                return true;
            }
        }
    }
    return false;
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

void DeviceBase::updateFirmware(const std::vector<uint8_t> &firmware, DeviceFwUpdateCallback updateCallback, bool async) {
    // todo: implement update firmware logic
    utils::unusedVar(firmware);
    utils::unusedVar(updateCallback);
    utils::unusedVar(async);
    throw invalid_value_exception("Do not support update firmware yet!");
}

}  // namespace libobsensor