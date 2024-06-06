#pragma once
#include "IDevice.hpp"

namespace libobsensor {

class IDeviceEnumerator; // forward declaration

struct DeviceEnumInfo {
    std::string connectionType_;  // "Ethernet", "USB2.0", "USB3.0", etc.

    // device identification info
    int         pid_ = 0;
    int         vid_ = 0;
    std::string uid_;  // Unique identifier of the port the device is connected to (pal specific)

    // device info
    std::string name_;
    std::string deviceSn_;

    // source port info list
    SourcePortInfoList sourcePortInfoList_;

    // device enumerator for creating device object
    std::weak_ptr<IDeviceEnumerator> deviceEnumerator_;

    virtual bool operator==(const DeviceEnumInfo &other) const {
        bool rst = (other.uid_ == uid_ && other.sourcePortInfoList_.size() == sourcePortInfoList_.size());
        if(rst && connectionType_ == "Ethernet") {
            auto netPort      = std::dynamic_pointer_cast<const NetSourcePortInfo>(sourcePortInfoList_.front());
            auto otherNetPort = std::dynamic_pointer_cast<const NetSourcePortInfo>(other.sourcePortInfoList_.front());
            rst &= (otherNetPort->address == netPort->address);
        }
        return rst;
    }
};

typedef std::function<void(std::vector<std::shared_ptr<DeviceEnumInfo>> removed, std::vector<std::shared_ptr<DeviceEnumInfo>> added)> DeviceChangedCallback;
typedef std::function<void(std::vector<std::shared_ptr<DeviceEnumInfo>> added)>                                                   DeviceInstalledCallback;
typedef std::function<void(std::vector<std::shared_ptr<DeviceEnumInfo>> removed)>                                                 DeviceRemovedCallback;

class IDeviceEnumerator {
public:
    virtual ~IDeviceEnumerator()                                                                              = default;
    virtual std::vector<std::shared_ptr<DeviceEnumInfo>> getDeviceInfoList()                                  = 0;
    virtual std::shared_ptr<IDevice>                 createDevice(std::shared_ptr<DeviceEnumInfo> info)       = 0;
    virtual void                                     setDeviceChangedCallback(DeviceChangedCallback callback) = 0;
};
}  // namespace libobsensor