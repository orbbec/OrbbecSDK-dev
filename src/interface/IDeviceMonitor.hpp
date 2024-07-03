#pragma once
#include "libobsensor/h/ObTypes.h"

#include <vector>
#include <functional>
#include <string>

namespace libobsensor {

typedef std::function<void(uint64_t stateCode, const std::string &message)> DeviceStateChangedCallback;

class IDeviceMonitor {
public:
    virtual ~IDeviceMonitor() = default;
    virtual void start()      = 0;
    virtual void stop()       = 0;

    virtual OBDeviceState getCurrentDeviceState() const                                     = 0;
    virtual int           registerStateChangedCallback(DeviceStateChangedCallback callback) = 0;
    virtual void          unregisterStateChangedCallback(int callbackId)                    = 0;

    virtual void enableHeartbeat()  = 0;
    virtual void disableHeartbeat() = 0;
    virtual void pauseHeartbeat()   = 0;
    virtual void resumeHeartbeat()  = 0;

    // for debug and vendor specific purpose
    virtual const std::vector<uint8_t> &sendAndReceiveData(const std::vector<uint8_t> &data, uint32_t exceptedRecvLen) = 0;
};
}  // namespace libobsensor