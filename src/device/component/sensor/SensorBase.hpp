#pragma once

#include "ISensor.hpp"
#include "ISourcePort.hpp"

#include <mutex>
#include <thread>
#include <condition_variable>

namespace libobsensor {

class SensorBase : public ISensor, public std::enable_shared_from_this<SensorBase> {
    static constexpr int DefaultNoStreamTimeoutMs        = 3000;
    static constexpr int DefaultStreamInterruptTimeoutMs = 3000;
    static constexpr int DefaultMaxRecoveryCount         = 3;

public:
    SensorBase(const std::shared_ptr<IDevice> &owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend);
    ~SensorBase() noexcept override;

    OBSensorType                 getSensorType() const override;
    std::shared_ptr<IDevice>     getOwner() const override;
    std::shared_ptr<ISourcePort> getBackend() const;

    OBStreamState getStreamState() const override;
    bool          isStreamActivated() const override;
    void          setStreamStateChangedCallback(StreamStateChangedCallback callback) override;

    StreamProfileList                    getStreamProfileList() const override;
    void                                 updateDefaultStreamProfile(const std::shared_ptr<const StreamProfile> &profile) override;
    std::shared_ptr<const StreamProfile> getActivatedStreamProfile() const override;

    FrameCallback getFrameCallback() const override;

    // when start Stream fails or interrupts, try to recover by restarting the stream; If Timeout<0, never timeout. if Timeout=0, use default timeout.
    void enableStreamRecovery(bool enable, uint32_t maxRecoveryCount = DefaultMaxRecoveryCount, int noStreamTimeoutMs = DefaultNoStreamTimeoutMs,
                              int streamInterruptTimeoutMs = DefaultStreamInterruptTimeoutMs);
    // stop trying to recover the stream
    void disableStreamRecovery();

protected:
    virtual void restartStream();
    virtual void updateStreamState(OBStreamState state);
    virtual void watchStreamState();

protected:
    const OBSensorType           sensorType_;
    std::weak_ptr<IDevice>       owner_;
    std::shared_ptr<ISourcePort> backend_;

    StreamProfileList streamProfileList_;

    std::shared_ptr<const StreamProfile> activatedStreamProfile_;
    FrameCallback                        frameCallback_;

    std::atomic<OBStreamState> streamState_;
    StreamStateChangedCallback streamStateChangedCallback_;

    std::mutex              streamStateMutex_;
    std::condition_variable streamStateCv_;
    std::thread             streamStateWatcherThread_;

    bool     onRecovering_;
    bool     recoveryEnabled_;
    uint32_t maxRecoveryCount_;
    uint32_t recoveryCount_;
    uint32_t noStreamTimeoutMs_;
    uint32_t streamInterruptTimeoutMs_;
};

}  // namespace libobsensor