#pragma once

#include "DeviceWatcher.hpp"
#include "ISourcePort.hpp"
#include "VendorNetDataPort.hpp"
#include "RTSPStreamPort.hpp"
#include "NetDataStreamPort.hpp"
#include "gige/GVCPClient.hpp"

#include <vector>

namespace libobsensor {

typedef std::vector<std::shared_ptr<NetSourcePortInfo>> NetSourcePortList;

class NetDeviceWatcher : public DeviceWatcher {
public:
    virtual ~NetDeviceWatcher() noexcept;
    virtual void start(deviceChangedCallback callback) override;
    virtual void stop() override;

private:
    deviceChangedCallback      callback_;
    std::thread                deviceWatchThread_;
    bool                       stopWatch_ = false;
    std::vector<NetDeviceInfo> netDevInfoList_;
    std::condition_variable    condVar_;
};

class Ethernet {
public:
    Ethernet(const Ethernet &)            = delete;
    Ethernet(Ethernet &&)                 = delete;
    Ethernet &operator=(const Ethernet &) = delete;
    Ethernet &operator=(Ethernet &&)      = delete;

    static Ethernet &instance() {
        static Ethernet instance;
        return instance;
    }

    SourcePortInfoList                 queryNetSourcePort();
    std::shared_ptr<NetSourcePortInfo> queryNetVendorPort(std::string address, uint16_t port);
    SourcePortInfoList                 queryAssociatedNetSourcePort(const std::shared_ptr<NetSourcePortInfo> info);

    bool changeNetDeviceIpConfig(std::string ip, const OBNetIpConfig &config);

    std::shared_ptr<DeviceWatcher> getDeviceWatcher() const {
        return std::make_shared<NetDeviceWatcher>();
    }

private:
    Ethernet() {}
    ~Ethernet() = default;

private:
    std::vector<NetDeviceInfo> netDeviceInfoList_;
    SourcePortInfoList         sourcePortInfoList_;
};

}  // namespace libobsensor
