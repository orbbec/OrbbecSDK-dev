
#include "StreamProfileFactory.hpp"
#include "exception/ObException.hpp"
#include "CoreTypeHelper.hpp"

namespace libobsensor {
namespace StreamProfileFactory {

std::shared_ptr<StreamProfile> createStreamProfile(OBStreamType streamType, OBFormat frameFormat) {
    if(streamType == OB_STREAM_UNKNOWN || frameFormat == OB_FORMAT_UNKNOWN) {
        throw invalid_value_exception("Invalid frame type or format");
    }
    switch(streamType) {
    case OB_STREAM_ACCEL:
        return createAccelStreamProfile( OB_ACCEL_FS_2g, OB_SAMPLE_RATE_1_5625_HZ);
    case OB_STREAM_GYRO:
        return createGyroStreamProfile(OB_GYRO_FS_16dps, OB_SAMPLE_RATE_1_5625_HZ);
    case OB_STREAM_VIDEO:
    case OB_STREAM_DEPTH:
    case OB_STREAM_COLOR:
    case OB_STREAM_IR:
    case OB_STREAM_IR_LEFT:
    case OB_STREAM_IR_RIGHT:
    case OB_STREAM_RAW_PHASE:
        return createVideoStreamProfile(streamType, frameFormat, 0, 0, 0);
    default:
        throw invalid_value_exception("Invalid stream type");
    }
}

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps) {
    return std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), type, format, width, height, fps);
}

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(std::weak_ptr<ISensor> owner, OBStreamType type, OBFormat format, uint32_t width, uint32_t height,
                                                             uint32_t fps) {
    return std::make_shared<VideoStreamProfile>(owner, type, format, width, height, fps);
}

std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(OBFormat format, OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate) {
    return std::make_shared<AccelStreamProfile>(std::weak_ptr<ISensor>(), fullScaleRange, sampleRate);
}

std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(std::weak_ptr<ISensor> owner, OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate) {
    return std::make_shared<AccelStreamProfile>(owner, fullScaleRange, sampleRate);
}

std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate) {
    return std::make_shared<GyroStreamProfile>(std::weak_ptr<ISensor>(), fullScaleRange, sampleRate);
}

std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(std::weak_ptr<ISensor> owner, OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate) {
    return std::make_shared<GyroStreamProfile>(owner, fullScaleRange, sampleRate);
}

}  // namespace StreamProfileFactory
}  // namespace libobsensor