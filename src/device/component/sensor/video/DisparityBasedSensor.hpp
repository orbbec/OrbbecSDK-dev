
#pragma once

#include "VideoSensor.hpp"

namespace libobsensor {
class DisparityBasedSensor : public VideoSensor {
public:
    DisparityBasedSensor(const std::shared_ptr<IDevice> &owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend);
    ~DisparityBasedSensor() override = default;

    void updateFormatFilterConfig(const std::vector<FormatFilterConfig> &configs) override;

private:
    void outputFrame(std::shared_ptr<Frame> frame) override;

    void convertProfileAsDisparityBasedProfile();
    void enableConvertOutputFrameAsDisparityFrame(bool enable);

private:
    bool convertOutputFrameAsDisparityFrame = false;
};
}  // namespace libobsensor