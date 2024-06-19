
#include "StreamProfileFactory.hpp"
#include "exception/ObException.hpp"

namespace libobsensor {
namespace StreamProfileFactory {

std::shared_ptr<StreamProfile> createStreamProfile(OBStreamType streamType) {
    return createStreamProfile(streamType, OB_FORMAT_ANY);
}

std::shared_ptr<StreamProfile> createStreamProfile(OBStreamType streamType, OBFormat frameFormat) {
    if(streamType == OB_STREAM_UNKNOWN || frameFormat == OB_FORMAT_UNKNOWN) {
        throw invalid_value_exception("Invalid frame type or format");
    }
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
    case OB_STREAM_DISPARITY:
        return createVideoStreamProfile(streamType, frameFormat, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);

    default:
        throw invalid_value_exception("Invalid stream type");
    }
}

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps,OBDeviceType deviceType) {
    return createVideoStreamProfile(std::shared_ptr<LazySensor>(), type, format, width, height, fps,deviceType);
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
    return std::make_shared<DisparityBasedStreamProfile>(videoProfile);
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

}  // namespace StreamProfileFactory
}  // namespace libobsensor