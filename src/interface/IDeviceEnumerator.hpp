#pragma once
#include "IDevice.hpp"
#include "ISourcePort.hpp"

namespace libobsensor {

class IDeviceEnumInfo {
public:
    virtual int                       getPid() const                = 0;
    virtual int                       getVid() const                = 0;
    virtual const std::string        &getUid() const                = 0;
    virtual const std::string        &getConnectionType() const     = 0;
    virtual const std::string        &getName() const               = 0;
    virtual const std::string        &getDeviceSn() const           = 0;
    virtual const SourcePortInfoList &getSourcePortInfoList() const = 0;
    virtual std::shared_ptr<IDevice> createDevice() const = 0;

    virtual bool operator==(const IDeviceEnumInfo &other) const = 0;
};

typedef std::vector<std::shared_ptr<const IDeviceEnumInfo>>                                     DeviceEnumInfoList;
typedef std::function<void(const DeviceEnumInfoList &removed, const DeviceEnumInfoList &added)> DeviceChangedCallback;

class IDeviceEnumerator {
public:
    virtual ~IDeviceEnumerator()                                                        = default;
    virtual DeviceEnumInfoList getDeviceInfoList()                                      = 0;
    virtual void               setDeviceChangedCallback(DeviceChangedCallback callback) = 0;
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