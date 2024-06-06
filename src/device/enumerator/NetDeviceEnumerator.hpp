#pragma once
#include "IDeviceEnumerator.hpp"
namespace libobsensor {
class NetDeviceEnumerator : public IDeviceEnumerator {
public:
    NetDeviceEnumerator(std::shared_ptr<pal::ObPal> obPal, DeviceChangedCallback callback);
    virtual ~NetDeviceEnumerator() noexcept;
    virtual std::vector<std::shared_ptr<DeviceEnumInfo>> getDeviceInfoList() override;
    virtual std::shared_ptr<IDevice>                 createDevice(std::shared_ptr<DeviceEnumInfo> info) override;
    virtual void                                     setDeviceChangedCallback(DeviceChangedCallback callback) override;

    static std::shared_ptr<IDevice> createDevice(std::shared_ptr<pal::ObPal> obPal, std::string address, uint16_t port);

private:
    static std::vector<std::shared_ptr<DeviceEnumInfo>> deviceInfoMatch(const SourcePortInfoList infoList);
    static std::shared_ptr<DeviceEnumInfo>              associatedSourcePortCompletion(std::shared_ptr<pal::ObPal> obPal, std::shared_ptr<DeviceEnumInfo> info);

    void                                     onPalDeviceChanged(pal::OBDeviceChangedType changeType, std::string devUid);
    std::vector<std::shared_ptr<DeviceEnumInfo>> queryDeviceList();

private:
    std::mutex            deviceChangedCallbackMutex_;
    DeviceChangedCallback deviceChangedCallback_;
    // std::thread           devChangedCallbackThread_;

    std::recursive_mutex                     deviceInfoListMutex_;
    std::vector<std::shared_ptr<DeviceEnumInfo>> deviceInfoList_;
    SourcePortInfoList                  sourcePortInfoList_;

    std::shared_ptr<pal::DeviceWatcher> deviceWatcher_;
};
}  // namespace libobsensor