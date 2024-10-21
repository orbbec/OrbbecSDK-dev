// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once

#include "VideoSensor.hpp"

namespace libobsensor {
class DisparityBasedSensor : public VideoSensor {
public:
    DisparityBasedSensor(IDevice *owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend);
    ~DisparityBasedSensor() override = default;

    void updateFormatFilterConfig(const std::vector<FormatFilterConfig> &configs) override;

    void markOutputDisparityFrame(bool enable);

    void setDepthUnit(float unit);
private:
    void outputFrame(std::shared_ptr<Frame> frame) override;

    void convertProfileAsDisparityBasedProfile();

    void syncDisparityToDepthModeStatus();

private:
    bool outputDisparityFrame_ = false;

    float depthUnit_ = 1.0f;
};
}  // namespace libobsensor
