#pragma once

#include "sensor/SensorBase.hpp"
#include "MotionStreamer.hpp"

namespace libobsensor {
class GyroSensor : public SensorBase {
public:
    GyroSensor(const std::shared_ptr<IDevice> &owner, const std::shared_ptr<ISourcePort> &backend, const std::shared_ptr<MotionStreamer> &streamer);
    ~GyroSensor() noexcept override;

    void start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) override;
    void stop() override;

private:
    std::shared_ptr<MotionStreamer> streamer_;
};
}  // namespace libobsensor