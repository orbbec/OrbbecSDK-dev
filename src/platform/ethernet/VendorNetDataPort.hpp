#pragma once

#include "ISourcePort.hpp"
#include "ethernet/socket/VendorTCPClient.hpp"

namespace libobsensor {
namespace pal {
class VendorNetDataPort : public IVendorDataPort {
public:
    VendorNetDataPort(std::shared_ptr<const NetSourcePortInfo> portInfo);
    virtual ~VendorNetDataPort() noexcept;
    virtual bool                                  sendData(const uint8_t *data, const uint32_t dataLen);
    virtual bool                                  recvData(uint8_t *data, uint32_t *dataLen);
    virtual std::shared_ptr<const SourcePortInfo> getSourcePortInfo() const;

private:
    std::shared_ptr<VendorTCPClient>         tcpClient_;
    std::shared_ptr<const NetSourcePortInfo> portInfo_;
};
}  // namespace pal
}  // namespace libobsensor