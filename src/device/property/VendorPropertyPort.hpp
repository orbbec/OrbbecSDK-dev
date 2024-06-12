#pragma once
#include "IProperty.hpp"
#include "ISourcePort.hpp"
#include <mutex>

namespace libobsensor {

class VendorPropertyPort : public IPropertyExtensionPort {
public:
    VendorPropertyPort(const std::shared_ptr<ISourcePort> &backend);
    ~VendorPropertyPort() noexcept override = default;

    void                        setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void                        getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void                        getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;
    void                        setFirmwareData(uint32_t propertyId, const std::vector<uint8_t> &data) override;
    const std::vector<uint8_t> &getFirmwareData(uint32_t propertyId) override;

private:
    void clearBuffers();

private:
    std::shared_ptr<ISourcePort> backend_;
    std::mutex                   mutex_;
    std::vector<uint8_t>         recvData_;
    std::vector<uint8_t>         sendData_;
    std::vector<uint8_t>         outputData_;
};

}  // namespace libobsensor
