#pragma once

#include "IProperty.hpp"
#include "IDevice.hpp"

#include <mutex>
#include <functional>

namespace libobsensor {
class DeviceComponentPropertyAccessor : public IBasicPropertyAccessor {
public:
    DeviceComponentPropertyAccessor(IDevice *device, DeviceComponentId compId);
    virtual ~DeviceComponentPropertyAccessor() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    IDevice          *device_;
    DeviceComponentId compId_;
};

class FunctionPropertyAccessor : public IBasicPropertyAccessor {
public:
    FunctionPropertyAccessor(std::function<OBPropertyValue(uint32_t)> getter, std::function<void(uint32_t, OBPropertyValue)> setter,
                             std::function<OBPropertyRange(uint32_t)> rangeGetter);

    virtual ~FunctionPropertyAccessor() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    std::function<OBPropertyValue(uint32_t)>       getter_;
    std::function<void(uint32_t, OBPropertyValue)> setter_;
    std::function<OBPropertyRange(uint32_t)>       rangeGetter_;
};

class LazyPropertyAccessor : public virtual IBasicPropertyAccessor {
public:
    LazyPropertyAccessor(std::function<std::shared_ptr<IPropertyAccessor>()> accessorCreator);

    virtual ~LazyPropertyAccessor() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

protected:
    std::function<std::shared_ptr<IPropertyAccessor>()> accessorCreator_;
    std::shared_ptr<IPropertyAccessor>                  accessor_;
    std::mutex                                          mutex_;
};

class LazyExtensionPropertyAccessor : public LazyPropertyAccessor, public IExtensionPropertyAccessor, public IExtensionPropertyAccessorV1_1 {
public:
    LazyExtensionPropertyAccessor(std::function<std::shared_ptr<IPropertyAccessor>()> accessorCreator);
    virtual ~LazyExtensionPropertyAccessor() noexcept = default;

    void                        setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) override;
    const std::vector<uint8_t> &getStructureData(uint32_t propertyId) override;
    void                        getRawData(uint32_t propertyId, GetDataCallback callback) override;

    uint16_t                    getCmdVersionProtoV1_1(uint32_t propertyId) override;
    const std::vector<uint8_t> &getStructureDataProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) override;
    void                        setStructureDataProtoV1_1(uint32_t propertyId, const std::vector<uint8_t> &data, uint16_t cmdVersion) override;
    const std::vector<uint8_t> &getStructureDataListProtoV1_1(uint32_t propertyId, uint16_t cmdVersion) override;
};

class HeartbeatPropertyAccessor : public IBasicPropertyAccessor {
public:
    HeartbeatPropertyAccessor(IDevice *owner);

    virtual ~HeartbeatPropertyAccessor() noexcept = default;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    IDevice *owner_;
};

class BaselinePropertyAccessor : public IExtensionPropertyAccessor {
public:
    BaselinePropertyAccessor(IDevice *owner);

    virtual ~BaselinePropertyAccessor() noexcept = default;

    void                        setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) override;
    const std::vector<uint8_t> &getStructureData(uint32_t propertyId) override;
    void                        getRawData(uint32_t propertyId, GetDataCallback callback) override;

private:
    IDevice             *owner_;
    std::vector<uint8_t> baselineData_;
};

}  // namespace libobsensor