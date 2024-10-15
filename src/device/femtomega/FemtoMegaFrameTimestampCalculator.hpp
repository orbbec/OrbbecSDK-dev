#pragma once

#include "timestamp/FrameTimestampCalculator.hpp"

namespace libobsensor {
class FemtoMegaColorFrameTimestampCalculatorV20300 : public FrameTimestampCalculatorBaseDeviceTime {
public:
    FemtoMegaColorFrameTimestampCalculatorV20300(IDevice *device, uint64_t deviceTimeFreq, uint64_t frameTimeFreq);

    virtual ~FemtoMegaColorFrameTimestampCalculatorV20300() = default;

    void calculate(std::shared_ptr<Frame> frame) override;
};
}  // namespace libobsensor