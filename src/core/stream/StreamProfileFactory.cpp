
#include "StreamProfileFactory.hpp"
#include "exception/ObException.hpp"

namespace libobsensor {
namespace StreamProfileFactory {

std::shared_ptr<StreamProfile> createStreamProfile(OBStreamType streamType, OBFormat frameFormat) {
    if(streamType == OB_STREAM_UNKNOWN || frameFormat == OB_FORMAT_UNKNOWN) {
        throw invalid_value_exception("Invalid frame type or format");
    }
    switch(streamType) {
    case OB_STREAM_ACCEL:
        return createAccelStreamProfile(OB_ACCEL_FS_2g, OB_SAMPLE_RATE_1_5625_HZ);
    case OB_STREAM_GYRO:
        return createGyroStreamProfile(OB_GYRO_FS_16dps, OB_SAMPLE_RATE_1_5625_HZ);
    case OB_STREAM_VIDEO:
    case OB_STREAM_DEPTH:
    case OB_STREAM_COLOR:
    case OB_STREAM_IR:
    case OB_STREAM_IR_LEFT:
    case OB_STREAM_IR_RIGHT:
    case OB_STREAM_RAW_PHASE:
    case OB_STREAM_DISPARITY:
        return createVideoStreamProfile(streamType, frameFormat, 0, 0, 0);

    default:
        throw invalid_value_exception("Invalid stream type");
    }
}

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps) {
    return std::make_shared<VideoStreamProfile>(std::shared_ptr<LazySensor>(), type, format, width, height, fps);
}

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(std::shared_ptr<LazySensor> owner, OBStreamType type, OBFormat format, uint32_t width,
                                                             uint32_t height, uint32_t fps) {
    return std::make_shared<VideoStreamProfile>(owner, type, format, width, height, fps);
}

std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate) {
    return std::make_shared<AccelStreamProfile>(std::shared_ptr<LazySensor>(), fullScaleRange, sampleRate);
}

std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(std::shared_ptr<LazySensor> owner, OBAccelFullScaleRange fullScaleRange,
                                                             OBAccelSampleRate sampleRate) {
    return std::make_shared<AccelStreamProfile>(owner, fullScaleRange, sampleRate);
}

std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate) {
    return std::make_shared<GyroStreamProfile>(std::shared_ptr<LazySensor>(), fullScaleRange, sampleRate);
}

std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(std::shared_ptr<LazySensor> owner, OBGyroFullScaleRange fullScaleRange,
                                                           OBGyroSampleRate sampleRate) {
    return std::make_shared<GyroStreamProfile>(owner, fullScaleRange, sampleRate);
}

}  // namespace StreamProfileFactory
}  // namespace libobsensor