#include "G330TimestampConverter.hpp"
#include "common/exception/ObException.hpp"

namespace libobsensor {

G330TimestampConverter::G330TimestampConverter(OBFrameMetadataType frameTimestampMetadataType, std::shared_ptr<GlobalTimestampFitter> globalTspFitter)
    : globalTspFitter_(globalTspFitter), frameTimestampMetadataType_(frameTimestampMetadataType) {}

void G330TimestampConverter::convert(const VideoFrameObject &srcFrame, std::shared_ptr<Frame> outFrame) {
    (void)srcFrame;
    uint64_t frameTsp = 0;
    try {
        frameTsp = outFrame->getMetadataValue(frameTimestampMetadataType_);
    }
    catch(...) {
    }

    outFrame->setTimeStampUs(frameTsp);
    calculateGlobalTimestamp(outFrame);
}

void G330TimestampConverter::convert(uint64_t srcTimestamp, std::shared_ptr<Frame> outFrame) {
    throw libobsensor::unsupported_operation_exception(
        "G330TimestampConverter::convert(uint64_t srcTimestamp, std::shared_ptr<Frame> outFrame) is not supported");
}

#define UINT32_T_OVERFLOW 0x100000000  // 32bit溢出值
void G330TimestampConverter::calculateGlobalTimestamp(std::shared_ptr<Frame> outFrame) {
    uint64_t       globalTsp          = 0;
    const uint32_t deviceClockFreq    = 1000;
    const uint32_t timestampClockFreq = 1000000;
    const uint64_t srcTimestamp       = outFrame->getTimeStampUs();

    if(globalTspFitter_) {
        auto   linearFuncParam = globalTspFitter_->getLinearFuncParam();
        double transformedTsp  = (double)srcTimestamp * deviceClockFreq / timestampClockFreq;
        globalTsp              = linearFuncParam.coefficientA * transformedTsp + linearFuncParam.constantB;
    }
    outFrame->setGlobalTimeStampUS(globalTsp);
}


}  // namespace libobsensor