#pragma once
#include "IDevice.hpp"
#include "ISourcePort.hpp"

namespace libobsensor {

class DeviceEnumInfo {
public:
    DeviceEnumInfo(int pid, int vid, const std::string &uid, const std::string &connectionType, const std::string &name, const std::string &deviceSn,
                   const SourcePortInfoList &sourcePortInfoList)
        : pid_(pid), vid_(vid), uid_(uid), connectionType_(connectionType), name_(name), deviceSn_(deviceSn), sourcePortInfoList_(sourcePortInfoList) {}
    DeviceEnumInfo(): pid_(0), vid_(0) {}

    virtual ~DeviceEnumInfo() = default;

    int getPid() const {
        return pid_;
    }

    int getVid() const {
        return vid_;
    }

    const std::string &getUid() const {
        return uid_;
    }

    const std::string &getConnectionType() const {
        return connectionType_;
    }

    const std::string &getName() const {
        return name_;
    }

    const std::string &getDeviceSn() const {
        return deviceSn_;
    }

    const SourcePortInfoList &getSourcePortInfoList() const {
        return sourcePortInfoList_;
    }

    bool operator==(const DeviceEnumInfo &other) const {
        bool rst = (other.uid_ == uid_ && other.sourcePortInfoList_.size() == sourcePortInfoList_.size());
        if(rst && connectionType_ == "Ethernet") {
            auto netPort      = std::dynamic_pointer_cast<const NetSourcePortInfo>(sourcePortInfoList_.front());
            auto otherNetPort = std::dynamic_pointer_cast<const NetSourcePortInfo>(other.sourcePortInfoList_.front());
            rst &= (otherNetPort->address == netPort->address);
        }
        return rst;
    }

    virtual std::shared_ptr<IDevice> createDevice() const = 0;

protected:
    // device identification info
    int         pid_;
    int         vid_;
    std::string uid_;  // Unique identifier of the port the device is connected to (pal specific)

    std::string connectionType_;  // "Ethernet", "USB2.0", "USB3.0", etc.

    // device info
    std::string name_;
    std::string deviceSn_;

    // source port info list
    SourcePortInfoList sourcePortInfoList_;
};

typedef std::vector<std::shared_ptr<const DeviceEnumInfo>>                                      DeviceEnumInfoList;
typedef std::function<void(const DeviceEnumInfoList &removed, const DeviceEnumInfoList &added)> DeviceChangedCallback;

class IDeviceEnumerator {
public:
    virtual ~IDeviceEnumerator()                                                                     = default;
    virtual DeviceEnumInfoList       getDeviceInfoList()                                             = 0;
    virtual void                     setDeviceChangedCallback(DeviceChangedCallback callback)        = 0;
};
}  // namespace libobsensor

#ifdef __cplusplus
extern "C" {
#endif
struct ob_device_list_t {
    libobsensor::DeviceEnumInfoList list;
};
#ifdef __cplusplus
}
#endif