#pragma once

#include "core/command/protocol/HostProtocol.hpp"
#define ISP_FLASH_ADDR 0x500000

namespace libobsensor {
namespace g2r {
class G330HostProtocol : public HostProtocol {
public:
    G330HostProtocol(std::shared_ptr<VendorDataPort> vendorDataPort, uint16_t flashPacketSize = FLASH_PAGE_SIZE * 3,
                    uint16_t flashEraseCmdDataSize = HP_ERASE_FLASH_CMD_DATA_SIZE, uint16_t rawDataPacketSize = FLASH_PAGE_SIZE * 3,
                    uint16_t deviceUpgradePacketSize = FLASH_PAGE_SIZE * 3)
        : HostProtocol(vendorDataPort, flashPacketSize, flashEraseCmdDataSize, rawDataPacketSize, deviceUpgradePacketSize) {}

    virtual ~G330HostProtocol() = default;

protected:
    HpStatus writeFlashFunc(uint32_t offset, const void *data, uint32_t dataLen, SetDataCallback callback) override;
    HpStatus updateFlashFunc(uint32_t offset, void *data, uint32_t dataLen, SetDataCallback callback) override;
};
}  // namespace g2r
}  // namespace libobsensor