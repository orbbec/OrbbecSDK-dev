#pragma once

#include "IFilter.hpp"
#include "ISourcePort.hpp"
#include "IDeviceComponent.hpp"

#include <atomic>
#include <map>
#include <mutex>

namespace libobsensor {

// Original imu data, software packaging method, needs to be calculated on the sdk side
typedef struct {
    uint8_t  reportId;    // Firmware fixed transmission 1
    uint8_t  sampleRate;  // OB_SAMPLE_RATE
    uint8_t  groupLen;    // sizeof(OBImuOriginData)
    uint8_t  groupCount;  // How many frames of data are in a packet.
    uint32_t reserved;    // reserve
} OBImuHeader;

typedef struct {
    int16_t  groupId;  // The number of groups in a pack
    int16_t  accelX;
    int16_t  accelY;
    int16_t  accelZ;
    int16_t  gyroX;
    int16_t  gyroY;
    int16_t  gyroZ;
    int16_t  temperature;
    uint32_t timestamp[2];
} OBImuOriginData;

class ImuStreamer : public IDeviceComponent {
public:
    ImuStreamer(IDevice *owner, const std::shared_ptr<IDataStreamPort> &backend, const std::shared_ptr<IFilter> &corrector);
    virtual ~ImuStreamer() noexcept;

    void start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback);
    void stop(std::shared_ptr<const StreamProfile> sp);

    IDevice *getOwner() const override;

private:
    virtual void praseIMUData(std::shared_ptr<Frame> frame);
    virtual void outputFrame(std::shared_ptr<Frame> frame);

private:
    IDevice                         *owner_;
    std::shared_ptr<IDataStreamPort> backend_;
    std::shared_ptr<IFilter>         corrector_;

    std::mutex                                                    cbMtx_;
    std::map<std::shared_ptr<const StreamProfile>, FrameCallback> callbacks_;

    std::atomic_bool running_;

    uint64_t frameIndex_;
};
}  // namespace libobsensor