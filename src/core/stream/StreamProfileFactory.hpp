#pragma once

#include "StreamProfile.hpp"

namespace libobsensor {
namespace StreamProfileFactory {

std::shared_ptr<StreamProfile> createStreamProfile(OBStreamType type, OBFormat frameFormat);

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps);
std::shared_ptr<DisparityStreamProfile> createDisparityStreamProfile(OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps);

std::shared_ptr<VideoStreamProfile> createVideoStreamProfile(std::weak_ptr<ISensor> owner, OBStreamType type, OBFormat format, uint32_t width, uint32_t height,
                                                             uint32_t fps);

                                                    

std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate);
std::shared_ptr<AccelStreamProfile> createAccelStreamProfile(std::weak_ptr<ISensor> owner, OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate);

std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate);
std::shared_ptr<GyroStreamProfile> createGyroStreamProfile(std::weak_ptr<ISensor> owner, OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate);

}  // namespace StreamProfileFactory
}  // namespace libobsensor