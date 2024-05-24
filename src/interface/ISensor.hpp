#pragma once
#include "openobsdk/h/ObTypes.h"
#include "stream/StreamProfile.hpp"
#include "frame/Frame.hpp"

#include <memory>

namespace libobsensor {
class IDevice;

typedef std::function<void(std::shared_ptr<Frame>)> FramePreProcessFunc;

class ISensor {
public:
    ISensor(std::weak_ptr<IDevice> owner) : owner_(owner){};
    virtual ~ISensor() noexcept = default;

    std::shared_ptr<IDevice> getOwner() {
        return owner_.lock();
    }

    using FrameCallback                                         = std::function<void(std::shared_ptr<Frame> frame)>;
    virtual void setFrameCallback(FrameCallback callback)       = 0;
    virtual void start(std::shared_ptr<const StreamProfile> sp) = 0;
    virtual void stop()                                         = 0;

    virtual OBSensorType                         getSensorType()           = 0;
    virtual StreamProfileList                    getStreamProfileList()    = 0;
    virtual std::shared_ptr<const StreamProfile> getCurrentStreamProfile() = 0;

    // When the device switches the camera depth mode, the sensor also needs to handle related work.
    virtual void handleDepthAlgModeChanged() = 0;
    virtual void handleDeviceDetached()      = 0;

    virtual bool isStreamStarted() = 0;

private:
    std::weak_ptr<IDevice> owner_;
};

}  // namespace libobsensor