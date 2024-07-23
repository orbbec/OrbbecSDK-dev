#include "IDeviceMonitor.hpp"
#include "ISourcePort.hpp"
#include "DeviceComponentBase.hpp"

#include <map>

namespace libobsensor {
class DeviceMonitor : public IDeviceMonitor, public DeviceComponentBase {
public:
    DeviceMonitor(IDevice *owner, std::shared_ptr<ISourcePort> sourcePort);
    virtual ~DeviceMonitor() noexcept;

    void start() override;
    void stop() override;

    OBDeviceState getCurrentDeviceState()  const override;
    int           registerStateChangedCallback(DeviceStateChangedCallback callback) override;
    void          unregisterStateChangedCallback(int callbackId) override;
    void          enableHeartbeat() override;
    void          disableHeartbeat() override;
    void          pauseHeartbeat() override;
    void          resumeHeartbeat() override;

    const std::vector<uint8_t> &sendAndReceiveData(const std::vector<uint8_t> &data, uint32_t exceptedRecvLen) override;

private:
    void heartbeatAndFetchState();
    void poll();

private:
    std::mutex                       commMutex_;
    std::shared_ptr<IVendorDataPort> vendorDataPort_;

    uint32_t                                       cbIdCounter_;
    std::mutex                                     stateChangedCallbacksMutex_;
    std::map<uint32_t, DeviceStateChangedCallback> stateChangedCallbacks_;

    std::atomic<bool> heartbeatEnabled_;
    std::atomic<bool> heartbeatPaused_;

    std::thread             heartbeatAndFetchStateThread_;
    std::atomic<bool>       heartbeatAndFetchStateThreadStarted_;
    std::condition_variable heartbeatAndFetchStateThreadCv_;

    std::mutex           recvDataMutex_;
    std::vector<uint8_t> recvData_;
    std::vector<uint8_t> sendData_;

    OBDeviceState devState_;
};
}  // namespace libobsensor