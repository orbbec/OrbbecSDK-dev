
#include "core/device/gemini2r/G330SensorStartStrategy.hpp"
#include "common/exception/ObException.hpp"
#include "common/logger/Logger.hpp"
#include "core/device/gemini2r/G330Device.hpp"


#include <vector>
#include <sstream>
#include <memory>

namespace libobsensor {

G330SensorStartStrategy::G330SensorStartStrategy(std::weak_ptr<IDevice> device) {
    initializeValidateSensor();
    device_ = device;

}

G330SensorStartStrategy::~G330SensorStartStrategy() noexcept {
    streamProfileMap_.clear();
}

void G330SensorStartStrategy::initializeValidateSensor() {
    sensorTypeList_.push_back(OB_SENSOR_DEPTH);
    sensorTypeList_.push_back(OB_SENSOR_IR_LEFT);
    sensorTypeList_.push_back(OB_SENSOR_IR_RIGHT);
}

bool G330SensorStartStrategy::validatePresetProfile(OBSensorType sensorType,std::shared_ptr<const StreamProfile> streamProfile) {
    bool isValid = true;
    auto device       = device_.lock();
    if(device == nullptr) {
        return isValid;
    }
    auto g2RDevice = std::dynamic_pointer_cast<G330Device>(device);
    auto        currentDepthMode = g2RDevice->getCurrentDepthAlgModeChecksum();
    const char *FactoryMode      = "Factory Calib";
    auto        depthMode        = currentDepthMode.name;
    if(strncmp(depthMode, FactoryMode, strlen(FactoryMode)) == 0)
   {
        switch(sensorType) {
        case OB_SENSOR_IR_LEFT:
        case OB_SENSOR_IR_RIGHT:
        {
            auto valVideoStreamProfile = streamProfile->as<VideoStreamProfile>();
            auto width  = valVideoStreamProfile->getWidth();
            auto height = valVideoStreamProfile->getHeight();
            auto format = valVideoStreamProfile->getFormat();
            //TODO:Gemini 330 Series Factory Calib only support 1280x800 Y12 and 640x400 Y12
            if(!(((width == 1280 && height == 800) || (width == 640 && height == 400)) && (format == OB_FORMAT_Y12 || format == OB_FORMAT_Y16))) {
                isValid = false;
            }
        }
            break;
        case OB_SENSOR_DEPTH:
            isValid = false;
            break;

        default:
            break;
        }
    }

    return isValid;
}

 bool G330SensorStartStrategy::validateSensorStart(OBSensorType sensorType, std::shared_ptr<const StreamProfile> streamProfile) {
    std::lock_guard<std::recursive_mutex> lock(validate_mutex_);
    auto                                  item = std::find(sensorTypeList_.begin(), sensorTypeList_.end(), sensorType);
    if(item == sensorTypeList_.end()) {
        // If sensorType does not exist, there is no need to verify the StreamProfile being started.
        return true;
    }

    if(streamProfileMap_.size() == 0) {
        streamProfileMap_[sensorType] = streamProfile;
        return true;
    }
    else {
        auto firElement   = streamProfileMap_.begin();
        auto firElementSp = firElement->second;

        auto firVideoStreamProfile = firElementSp->as<VideoStreamProfile>();
        auto valVideoStreamProfile = streamProfile->as<VideoStreamProfile>();

        if(firVideoStreamProfile->getFps() == valVideoStreamProfile->getFps() && firVideoStreamProfile->getWidth() == valVideoStreamProfile->getWidth()
           && firVideoStreamProfile->getHeight() == valVideoStreamProfile->getHeight()) {
            streamProfileMap_[sensorType] = streamProfile;
            return true;
        }
        else {
            LOG_ERROR("G330 device start {} resolution {}x{}, {}fps, which is different from the {} start resolution {}x{}, {}fps.", sensorType,
                      valVideoStreamProfile->getWidth(), valVideoStreamProfile->getHeight(), valVideoStreamProfile->getFps(), firElement->first,
                      firVideoStreamProfile->getWidth(), firVideoStreamProfile->getHeight(), firVideoStreamProfile->getFps());
        }
    }

    return false;
}

void G330SensorStartStrategy::clearValidateSensor(OBSensorType sensorType) {
    std::lock_guard<std::recursive_mutex> lock(validate_mutex_);
    auto                                  it = streamProfileMap_.find(sensorType);
    if(it != streamProfileMap_.end()) {
        streamProfileMap_.erase(sensorType);
    }
}


}  // namespace libobsensor