// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once
#include "sensor/SensorBase.hpp"

namespace libobsensor {
class VideoSensor : public SensorBase {
public:
    VideoSensor(const std::shared_ptr<IDevice>& owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort>& backend);

    ~VideoSensor() noexcept override = default;

    void start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) override;
    void stop() override;
};

}  // namespace libobsensor
