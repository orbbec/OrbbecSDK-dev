#pragma once
#include "IDeviceEnumerator.hpp"
#include "DeviceWatcher.hpp"
#include "ObPal.hpp"

namespace libobsensor {
class UsbDeviceEnumerator : public IDeviceEnumerator {
public:
    UsbDeviceEnumerator(DeviceChangedCallback callback);
    ~UsbDeviceEnumerator() noexcept override;
    virtual std::vector<std::shared_ptr<DeviceEnumInfo>> getDeviceInfoList() override;
    virtual std::shared_ptr<IDevice>                 createDevice(std::shared_ptr<DeviceEnumInfo> info) override;
    virtual void                                     setDeviceChangedCallback(DeviceChangedCallback callback) override;

private:
    void                                     onPalDeviceChanged(pal::OBDeviceChangedType changeType, std::string devUid);
    std::vector<std::shared_ptr<DeviceEnumInfo>> queryRemovedDevice(std::string rmDevUid);
    std::vector<std::shared_ptr<DeviceEnumInfo>> queryArrivalDevice();

    void deviceArrivalHandleThreadFunc();

    static std::vector<std::shared_ptr<DeviceEnumInfo>> usbDeviceInfoMatch(const SourcePortInfoList infoList);

private:
    std::shared_ptr<pal::ObPal> obPal_;
    bool destroy_ = false;

    std::shared_ptr<pal::DeviceWatcher> deviceWatcher_;

    DeviceChangedCallback devChangedCallback_ = nullptr;
    std::thread           devChangedCallbackThread_;

    SourcePortInfoList currentUsbPortInfoList_;
    bool                    newUsbPortArrival_ = false;
    std::condition_variable newUsbPortArrivalCV_;
    std::thread             deviceArrivalHandleThread_;

    std::vector<std::shared_ptr<DeviceEnumInfo>> deviceInfoList_;
    std::recursive_mutex                     deviceInfoListMutex_;

    std::mutex callbackMutex_;

};
}  // namespace libobsensor