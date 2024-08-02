#pragma once
#include "IFrame.hpp"
#include "timestamp/GlobalTimestampFilter.hpp"

namespace libobsensor {

class G330TimestampCalculator : public IFrameTimestampCalculator {
public:
    G330TimestampCalculator(OBFrameMetadataType frameTimestampMetadataType, std::shared_ptr<GlobalTimestampFilter> globalTspFitter);
    ~G330TimestampCalculator() override = default;

    void calculate(std::shared_ptr<Frame> frame) override;

    void clear() override {};

private:
    void calculateGlobalTimestamp(std::shared_ptr<Frame> frame);

private:
    std::shared_ptr<GlobalTimestampFilter> globalTspFilter_;

    const OBFrameMetadataType frameTimestampMetadataType_;
};

}  // namespace libobsensor