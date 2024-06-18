#pragma once
#include "IProperty.hpp"
#include "ISourcePort.hpp"
#include <mutex>

namespace libobsensor {
class VendorPropertyPort : public IPropertyExtensionPort, public IPropertyExtensionPortV1_1 {  // todo: split into two classes
public:
    explicit VendorPropertyPort(const std::shared_ptr<ISourcePort> &backend);
    ~VendorPropertyPort() noexcept override = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

    void                        setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data);
    const std::vector<uint8_t> &getStructureData(uint32_t propertyId) override;

    void getRawData(uint32_t propertyId, GetDataCallback callback) override;

    uint16_t                    getCmdVersionProtoV1_1(uint32_t propertyId) override;
    const std::vector<uint8_t> &getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) override;
    const std::vector<uint8_t> &getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) override;

private:
    void clearBuffers();

private:
    std::shared_ptr<ISourcePort>      backend_;
    std::mutex                        mutex_;
    std::vector<uint8_t>              recvData_;
    std::vector<uint8_t>              sendData_;
    std::vector<uint8_t>              outputData_;
    std::vector<std::vector<uint8_t>> structureDataList_;  // for cmd version 1.1
};
}  // namespace libobsensor
