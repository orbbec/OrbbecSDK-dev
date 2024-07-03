
#include "DisparityBasedSensor.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "frame/Frame.hpp"

namespace libobsensor {
DisparityBasedSensor::DisparityBasedSensor(IDevice *owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend)
    : VideoSensor(owner, sensorType, backend) {
    convertProfileAsDisparityBasedProfile();
}

void DisparityBasedSensor::updateFormatFilterConfig(const std::vector<FormatFilterConfig> &configs) {
    VideoSensor::updateFormatFilterConfig(configs);
    convertProfileAsDisparityBasedProfile();
}

void DisparityBasedSensor::convertProfileAsDisparityBasedProfile() {
    auto oldSpList = streamProfileList_;
    streamProfileList_.clear();
    for(auto &sp: oldSpList) {
        auto vsp   = sp->as<const VideoStreamProfile>();
        auto newSp = StreamProfileFactory::createDisparityBasedStreamProfile(vsp);
        auto iter = streamProfileBackendMap_.find(sp);
        streamProfileBackendMap_.insert({newSp,{iter->second.first, iter->second.second}});
        streamProfileList_.push_back(newSp);
    }
}

void DisparityBasedSensor::enableConvertOutputFrameAsDisparityFrame(bool enable) {
    convertOutputFrameAsDisparityFrame = enable;
}

void DisparityBasedSensor::outputFrame(std::shared_ptr<Frame> frame) {
    if(convertOutputFrameAsDisparityFrame) {
        auto sp    = frame->getStreamProfile();
        auto newSp = sp->clone();
        newSp->setFormat(OB_FORMAT_DISP16);
    }
    VideoSensor::outputFrame(frame);
}

}  // namespace libobsensor