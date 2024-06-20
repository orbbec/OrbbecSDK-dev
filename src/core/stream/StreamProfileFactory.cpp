
#include "StreamProfileFactory.hpp"
#include "StreamIntrinsicsManager.hpp"
#include "StreamExtrinsicsManager.hpp"
#include "exception/ObException.hpp"
#include "envconfig/EnvConfig.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {
namespace StreamProfileFactory {

std::shared_ptr<StreamProfile> createStreamProfile(OBStreamType streamType) {
    return createStreamProfile(streamType, OB_FORMAT_ANY);
}

std::shared_ptr<StreamProfile> createStreamProfile(OBStreamType streamType, OBFormat frameFormat) {
    switch(streamType) {
    case OB_STREAM_ACCEL:
        return createAccelStreamProfile(OB_ACCEL_FULL_SCALE_RANGE_ANY, OB_ACCEL_SAMPLE_RATE_ANY);
    case OB_STREAM_GYRO:
        return createGyroStreamProfile(OB_GYRO_FULL_SCALE_RANGE_ANY, OB_GYRO_SAMPLE_RATE_ANY);
    case OB_STREAM_VIDEO:
    case OB_STREAM_DEPTH:
    case OB_STREAM_COLOR:
    case OB_STREAM_IR:
    case OB_STREAM_IR_LEFT:
    case OB_STREAM_IR_RIGHT:
    case OB_STREAM_RAW_PHASE:
        return createVideoStreamProfile(streamType, frameFormat, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);

    default:
        return std::make_shared<StreamProfile>(std::shared_ptr<LazySensor>(), streamType, frameFormat);
    }
}

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps,
                                                             OBDeviceType deviceType) {
    return createVideoStreamProfile(std::shared_ptr<LazySensor>(), type, format, width, height, fps, deviceType);
}

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(std::shared_ptr<LazySensor> owner, OBStreamType type, OBFormat format, uint32_t width,
                                                             uint32_t height, uint32_t fps, OBDeviceType deviceType) {
    std::shared_ptr<VideoStreamProfile> vsp;
    if(type == OB_STREAM_DEPTH && (deviceType == OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA || deviceType == OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA)) {
        vsp = std::make_shared<DisparityBasedStreamProfile>(owner, type, format, width, height, fps);
    }
    else {
        vsp = std::make_shared<VideoStreamProfile>(owner, type, format, width, height, fps);
    }

    vsp->bindIntrinsic({ 0 });
    vsp->bindDistortion({ 0 });
    return vsp;
}

std::shared_ptr<DisparityBasedStreamProfile> createDisparityBasedStreamProfile(std::shared_ptr<const VideoStreamProfile> videoProfile) {
    auto vsp           = std::make_shared<DisparityBasedStreamProfile>(videoProfile->getOwner(), videoProfile->getType(), videoProfile->getFormat(),
                                                                       videoProfile->getWidth(), videoProfile->getHeight(), videoProfile->getFps());
    auto intrinsicsMgr = StreamIntrinsicsManager::getInstance();
    if(intrinsicsMgr->containsVideoStreamIntrinsics(videoProfile)) {
        auto intrinsics = intrinsicsMgr->getVideoStreamIntrinsics(videoProfile);
        vsp->bindIntrinsic(intrinsics);
    }
    if(intrinsicsMgr->containsVideoStreamDistortion(videoProfile)) {
        auto distortion = intrinsicsMgr->getVideoStreamDistortion(videoProfile);
        vsp->bindDistortion(distortion);
    }
    auto extrinsicsMgr = StreamExtrinsicsManager::getInstance();
    extrinsicsMgr->registerSameExtrinsics(vsp, videoProfile);
    return vsp;
}

std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate) {
    return createAccelStreamProfile(std::shared_ptr<LazySensor>(), fullScaleRange, sampleRate);
}

std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(std::shared_ptr<LazySensor> owner, OBAccelFullScaleRange fullScaleRange,
                                                             OBAccelSampleRate sampleRate) {
    auto asp = std::make_shared<AccelStreamProfile>(owner, fullScaleRange, sampleRate);
    asp->bindIntrinsic({ 0 });
    return asp;
}

std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate) {
    return createGyroStreamProfile(std::shared_ptr<LazySensor>(), fullScaleRange, sampleRate);
}

std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(std::shared_ptr<LazySensor> owner, OBGyroFullScaleRange fullScaleRange,
                                                           OBGyroSampleRate sampleRate) {
    auto gsp = std::make_shared<GyroStreamProfile>(owner, fullScaleRange, sampleRate);
    gsp->bindIntrinsic({ 0 });
    return gsp;
}

std::shared_ptr<const StreamProfile> getDefaultStreamProfileFormEnvConfig(const std::string &deviceName, OBSensorType sensorType) {
    if(sensorType == OB_SENSOR_ACCEL || sensorType == OB_SENSOR_GYRO) {
        // todo: add support for accel/gyro default stream profile
        throw std::runtime_error("Default stream profile for accel/gyro is not supported");
    }

    auto devName = utils::remove(deviceName, " ");

    auto        envConfig    = EnvConfig::getInstance();
    std::string nodePathName = "Device." + devName + ".";
    nodePathName += utils::getSensorName(sensorType);
    int         w = 0, h = 0, fps = 0;
    std::string fmt;
    if(envConfig->getIntValue(nodePathName + ".Width", w) && envConfig->getIntValue(nodePathName + ".Height", h)
       && envConfig->getIntValue(nodePathName + ".FPS", fps) && envConfig->getStringValue(nodePathName + ".Format", fmt)) {
        auto width      = static_cast<uint32_t>(w);
        auto height     = static_cast<uint32_t>(h);
        auto frameRate  = static_cast<uint32_t>(fps);
        auto format     = utils::strToOBFormat(fmt);
        auto streamType = utils::mapSensorTypeToStreamType(sensorType);
        return StreamProfileFactory::createVideoStreamProfile(streamType, format, width, height, frameRate);
    }
    return nullptr;
}

}  // namespace StreamProfileFactory
}  // namespace libobsensor