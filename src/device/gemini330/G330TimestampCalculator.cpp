#include "G330TimestampCalculator.hpp"
#include "exception/ObException.hpp"
#include "frame/Frame.hpp"

namespace libobsensor {

G330TimestampCalculator::G330TimestampCalculator(OBFrameMetadataType frameTimestampMetadataType, std::shared_ptr<GlobalTimestampFitter> globalTspFitter)
    : globalTspFitter_(globalTspFitter), frameTimestampMetadataType_(frameTimestampMetadataType) {}

void G330TimestampCalculator::calculate(std::shared_ptr<Frame> frame) {
    uint64_t frameTsp = 0;
    try {
        frameTsp = frame->getMetadataValue(frameTimestampMetadataType_);
    }
    catch(...) {
    }

    frame->setTimeStampUsec(frameTsp);
    calculateGlobalTimestamp(frame);

#if 1
    auto globalTsp = frame->getGlobalTimeStampUsec();
    auto frameType = frame->getType();
    LOG_ERROR("Frame {} timestamp: {} us, Global timestamp: {} us", frameType, frameTsp, globalTsp);
#endif
}

void G330TimestampCalculator::calculateGlobalTimestamp(std::shared_ptr<Frame> frame) {
    uint64_t       globalTsp          = 0;
    const uint32_t deviceClockFreq    = 1000;
    const uint32_t timestampClockFreq = 1000000;
    const uint64_t srcTimestamp       = frame->getTimeStampUsec();

    if(globalTspFitter_) {
        auto   linearFuncParam = globalTspFitter_->getLinearFuncParam();
        double transformedTsp  = (double)srcTimestamp * deviceClockFreq / timestampClockFreq;
        globalTsp              = static_cast<uint64_t>(linearFuncParam.coefficientA * transformedTsp + linearFuncParam.constantB);
    }
    frame->setGlobalTimeStampUsec(globalTsp);
}

}  // namespace libobsensor