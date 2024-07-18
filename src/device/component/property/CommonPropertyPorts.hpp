#pragma once

#include "IDevice.hpp"
#include "IProperty.hpp"

#include <mutex>
namespace libobsensor {
class DeviceComponentPropertyPortWrapper : public IPropertyPort {
public:
    DeviceComponentPropertyPortWrapper(IDevice *device, const std::string &compName);
    virtual ~DeviceComponentPropertyPortWrapper() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    IDevice    *device_;
    std::string compName_;
};

class FunctionPropertyPortWrapper : public IPropertyPort {
public:
    FunctionPropertyPortWrapper(std::function<OBPropertyValue(uint32_t)> getter, std::function<void(uint32_t, OBPropertyValue)> setter,
                                std::function<OBPropertyRange(uint32_t)> rangeGetter);

    virtual ~FunctionPropertyPortWrapper() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    std::function<OBPropertyValue(uint32_t)>       getter_;
    std::function<void(uint32_t, OBPropertyValue)> setter_;
    std::function<OBPropertyRange(uint32_t)>       rangeGetter_;
};

class LazyPropertyPortWrapper : public virtual IPropertyPort {
public:
    LazyPropertyPortWrapper(std::function<std::shared_ptr<IPropertyPort>()> portCreator);

    virtual ~LazyPropertyPortWrapper() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

protected:
    std::function<std::shared_ptr<IPropertyPort>()> portCreator_;
    std::shared_ptr<IPropertyPort>                  port_;
    std::mutex                                      mutex_;
};

class LazyPropertyExtensionPortWrapper : public LazyPropertyPortWrapper, public virtual IPropertyExtensionPort, public virtual IPropertyExtensionPortV1_1 {
public:
    LazyPropertyExtensionPortWrapper(std::function<std::shared_ptr<IPropertyExtensionPort>()> portCreator);
    virtual ~LazyPropertyExtensionPortWrapper() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override {
        LazyPropertyPortWrapper::setPropertyValue(propertyId, value);
    }

    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override {
        LazyPropertyPortWrapper::getPropertyValue(propertyId, value);
    }

    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override {
        LazyPropertyPortWrapper::getPropertyRange(propertyId, range);
    }

    void                        setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) override;
    const std::vector<uint8_t> &getStructureData(uint32_t propertyId) override;
    void                        getRawData(uint32_t propertyId, GetDataCallback callback) override;

    uint16_t                    getCmdVersionProtoV1_1(uint32_t propertyId) override;
    const std::vector<uint8_t> &getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) override;
    void                        setStructureDataProtoV1_1(uint32_t propertyId, const std::vector<uint8_t> &data, uint16_t cmdVersion) override;
    const std::vector<uint8_t> &getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) override;
};

}  // namespace libobsensor