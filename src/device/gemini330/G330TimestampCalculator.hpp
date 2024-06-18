#pragma once
#include "IFrame.hpp"
#include "timestamp/GlobalTimestampFitter.hpp"

namespace libobsensor {

class G330TimestampCalculator : public IFrameTimestampCalculator {
public:
    G330TimestampCalculator(OBFrameMetadataType frameTimestampMetadataType, std::shared_ptr<GlobalTimestampFitter> globalTspFitter);
    ~G330TimestampCalculator() override = default;

    void calculate(std::shared_ptr<Frame> frame) override;

    void clear() override {};

private:
    void calculateGlobalTimestamp(std::shared_ptr<Frame> frame);

private:
    std::shared_ptr<GlobalTimestampFitter> globalTspFitter_;

    const OBFrameMetadataType frameTimestampMetadataType_;
};

}  // namespace libobsensor