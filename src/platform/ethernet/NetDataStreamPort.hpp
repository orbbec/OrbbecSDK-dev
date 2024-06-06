#pragma once
#include "ISourcePort.hpp"
#include "ethernet/socket/VendorTCPClient.hpp"
#include <set>
#include <thread>
#include <mutex>

namespace libobsensor {
namespace pal {

struct NetDataStreamPortInfo : public NetSourcePortInfo {
    NetDataStreamPortInfo(std::string address, uint16_t port, uint16_t vendorPort, std::string mac = "unknown", std::string serialNumber = "unknown",
                          uint32_t pid = 0)
        : NetSourcePortInfo(SOURCE_PORT_NET_VENDOR_STREAM, address, port, mac, serialNumber, pid), vendorPort(vendorPort) {}

    virtual bool equal(std::shared_ptr<const SourcePortInfo> cmpInfo) const override {
        if(cmpInfo->portType != portType) {
            return false;
        }
        auto netCmpInfo = std::dynamic_pointer_cast<const NetDataStreamPortInfo>(cmpInfo);
        return (address == netCmpInfo->address) && (port == netCmpInfo->port) && (vendorPort == netCmpInfo->vendorPort);
    };

    uint16_t vendorPort;
};

class NetDataStreamPort : public IDataStreamPort {
public:
    NetDataStreamPort(std::shared_ptr<const NetDataStreamPortInfo> portInfo);
    virtual ~NetDataStreamPort() noexcept;
    virtual void                                  addWatcher(std::weak_ptr<DataStreamWatcher> watcher) override;
    virtual void                                  removeWatcher(std::weak_ptr<DataStreamWatcher> watcher) override;
    virtual std::shared_ptr<const SourcePortInfo> getSourcePortInfo() const override {
        return portInfo_;
    }

public:
    void readData();

private:
    std::shared_ptr<const NetDataStreamPortInfo> portInfo_;

    std::set<std::weak_ptr<DataStreamWatcher>, dataStreamWatcherWeakPtrCompare> watchers_;
    std::mutex                                                                  watchersMutex_;

    std::shared_ptr<VendorTCPClient> tcpClient_;
    std::mutex                       clientMutex_;
    std::thread                      readDataThread_;

    bool isStreaming_;
};

}  // namespace pal
}  // namespace libobsensor