#pragma once
#include "interface/IFrameTimestampConverter.hpp"
#include "core/device/component/GlobalTimestampFitter.hpp"

namespace libobsensor {
namespace g2r {
class G330TimestampConverter : public IFrameTimestampConverter {
public:
    G330TimestampConverter(OBFrameMetadataType frameTimestampMetadataType, std::shared_ptr<GlobalTimestampFitter> globalTspFitter);
    virtual ~G330TimestampConverter() = default;

    virtual void convert(const VideoFrameObject &srcFrame, std::shared_ptr<Frame> outFrame) override;
    virtual void convert(uint64_t srcTimestamp, std::shared_ptr<Frame> outFrame) override;
    virtual void clear() override{};

private:
    void calculateGlobalTimestamp(std::shared_ptr<Frame> outFrame);

private:
    std::shared_ptr<GlobalTimestampFitter> globalTspFitter_;

    const OBFrameMetadataType frameTimestampMetadataType_;
};
}  // namespace g2r
}  // namespace libobsensor