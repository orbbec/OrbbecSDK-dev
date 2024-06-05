#include "IMUFrameReversion.hpp"
#include "frame/Frame.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "openobsdk/h/ObTypes.h"

namespace libobsensor {

IMUFrameReversion::IMUFrameReversion(const std::string &name) : FilterBase(name) {
    srcFrameQueue_ = std::make_shared<FrameQueue<const Frame>>(100);
    LOG_DEBUG("IMUFrameReversion Filter {} created with frame queue capacity {}", name_, srcFrameQueue_->size());
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

    auto outFrame = FrameFactory::cloneFrame(frame);
    if(frame->getType() == OB_FRAME_ACCEL) {
        AccelFrame::OBAccelFrameData *frameData = (AccelFrame::OBAccelFrameData *)outFrame->getData();
        frameData->accelData[0] *= -1;
        frameData->accelData[1] *= -1;
        frameData->accelData[2] *= -1;
        return outFrame;
    }
    else if(frame->getType() == OB_FRAME_GYRO) {
        GyroFrame::OBGyroFrameData *gyroFrameData = (GyroFrame::OBGyroFrameData *)outFrame->getData();
        gyroFrameData->gyroData[0] *= -1;
        gyroFrameData->gyroData[1] *= -1;
        gyroFrameData->gyroData[2] *= -1;
        return outFrame;
    }

    return outFrame;
}

}  // namespace libobsensor
