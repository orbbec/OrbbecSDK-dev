#pragma once
#include "IProperty.hpp"
#include "ISourcePort.hpp"
#include <mutex>

namespace libobsensor {
template <typename T, uint16_t CMD_VER> 
class VendorPropertyPort : public IPropertyExtensionPort<T, CMD_VER> {
public:
    VendorPropertyPort(const std::shared_ptr<ISourcePort> &backend);
    ~VendorPropertyPort() noexcept override = default;

    void                        setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void                        getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void                        getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;
    void                        setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) override;
    const std::vector<uint8_t> &getStructureData(uint32_t propertyId) override;
    void                        getCmdVersionProtoV11(uint32_t propertyId, uint16_t *version) override;
    void                        getRawData(uint32_t propertyId, get_data_callback callback, uint32_t transPacketSize) override;
    T                           getStructureDataProtoV11(uint32_t propertyId) override;
    T                           getStructureDataListProtoV11(uint32_t propertyId, uint32_t tranPacketSize) override;

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
