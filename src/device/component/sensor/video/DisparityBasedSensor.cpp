
#include "DisparityBasedSensor.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "frame/Frame.hpp"
#include "IProperty.hpp"

namespace libobsensor {
DisparityBasedSensor::DisparityBasedSensor(IDevice *owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend)
    : VideoSensor(owner, sensorType, backend) {
    convertProfileAsDisparityBasedProfile();
    syncDisparityToDepthModeStatus(); 
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

void DisparityBasedSensor::setDepthUnit(float unit){
    depthUnit_ = unit;
}

void DisparityBasedSensor::outputFrame(std::shared_ptr<Frame> frame) {
    if(convertOutputFrameAsDisparityFrame) {
        auto sp    = frame->getStreamProfile();
        auto newSp = sp->clone();
        newSp->setFormat(OB_FORMAT_DISP16);
        frame->setStreamProfile(newSp);
    }

    auto depthFrame = frame->as<DepthFrame>();
    if(depthFrame) {
        depthFrame->setValueScale(depthUnit_);
    }
    
    VideoSensor::outputFrame(frame);
}

void DisparityBasedSensor::syncDisparityToDepthModeStatus(){
    OBPropertyValue hwDisparityValue;
    OBPropertyValue swDisparityValue;
    auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
    commandPort->getPropertyValue(OB_PROP_DISPARITY_TO_DEPTH_BOOL,&hwDisparityValue);
    auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
    processor->getPropertyValue(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, &swDisparityValue);
    if(hwDisparityValue.intValue == 1 && swDisparityValue.intValue == 1){
        swDisparityValue.intValue = 0;
        processor->setPropertyValue(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, swDisparityValue);
    }
}

}  // namespace libobsensor