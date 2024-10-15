#include "FemtoMegaFrameTimestampCalculator.hpp"
#include "frame/Frame.hpp"

namespace libobsensor {
FemtoMegaColorFrameTimestampCalculatorV20300::FemtoMegaColorFrameTimestampCalculatorV20300(IDevice *device, uint64_t deviceTimeFreq, uint64_t frameTimeFreq)
    : FrameTimestampCalculatorBaseDeviceTime(device, deviceTimeFreq, frameTimeFreq) {}

void FemtoMegaColorFrameTimestampCalculatorV20300::calculate(std::shared_ptr<Frame> frame) {
    auto format = frame->getFormat();
    if(format == OB_FORMAT_MJPEG) {
        return;
    }
    FrameTimestampCalculatorBaseDeviceTime::calculate(frame);
}

}  // namespace libobsensor