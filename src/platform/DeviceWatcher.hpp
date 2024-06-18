#pragma once
#include <functional>
#include <string>

namespace libobsensor {

typedef enum {
    OB_DEVICE_REMOVED,
    OB_DEVICE_ARRIVAL,
} OBDeviceChangedType;

typedef std::function<void(OBDeviceChangedType changeType, std::string devUid)> deviceChangedCallback;

class DeviceWatcher {
public:
    virtual ~DeviceWatcher() noexcept = default;

    virtual void start(deviceChangedCallback callback) = 0;
    virtual void stop()                                = 0;
};

}  // namespace libobsensor