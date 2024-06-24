#include "IMUFrameReversion.hpp"
#include "frame/Frame.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "libobsensor/h/ObTypes.h"

namespace libobsensor {

IMUFrameReversion::IMUFrameReversion(const std::string &name) : FilterBase(name) {
    srcFrameQueue_ = std::make_shared<FrameQueue<const Frame>>(100);
    LOG_DEBUG("IMUFrameReversion {} created with frame queue capacity {}", name_, srcFrameQueue_->capacity());
}
IMUFrameReversion::~IMUFrameReversion() noexcept {}

void IMUFrameReversion::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 0) {
        throw unsupported_operation_exception("IMUFrameReversion update config error: unsupported operation.");
    }
}

const std::string &IMUFrameReversion::getConfigSchema() const {
    throw unsupported_operation_exception("IMUFrameReversion get config schema error: unsupported operation.");
}

std::shared_ptr<Frame> IMUFrameReversion::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    auto newFrame = FrameFactory::cloneFrame(frame, true);
    if(!frame->is<FrameSet>()) {
        return newFrame;
    }

    auto frameSet   = newFrame->as<FrameSet>();
    auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
    if(accelFrame) {
        AccelFrame::OBAccelFrameData *frameData = (AccelFrame::OBAccelFrameData *)accelFrame->getData();
        frameData->accelData[0] *= -1;
        frameData->accelData[1] *= -1;
        frameData->accelData[2] *= -1;
    }

    auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
    if(gyroFrame) {
        GyroFrame::OBGyroFrameData *gyroFrameData = (GyroFrame::OBGyroFrameData *)gyroFrame->getData();
        gyroFrameData->gyroData[0] *= -1;
        gyroFrameData->gyroData[1] *= -1;
        gyroFrameData->gyroData[2] *= -1;
    }

    return newFrame;
}

}  // namespace libobsensor
