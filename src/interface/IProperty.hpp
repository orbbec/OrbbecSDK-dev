#pragma once
#include <vector>
#include <memory>
#include "openobsdk/h/ObTypes.h"
#include "exception/ObException.hpp"

namespace libobsensor {
typedef union {
    int32_t intValue;
    float   floatValue;
} OBPropertyValue;

typedef struct {
    OBPropertyValue cur;
    OBPropertyValue max;
    OBPropertyValue min;
    OBPropertyValue step;
    OBPropertyValue def;
} OBPropertyRange;

template <typename T> struct OBPropertyRangeT {
    T cur;
    T max;
    T min;
    T step;
    T def;
};

class IPropertyPort {
public:
    virtual ~IPropertyPort() noexcept                                          = default;
    virtual void setPropertyValue(uint64_t propertyId, OBPropertyValue value)  = 0;
    virtual void getPropertyValue(uint64_t propertyId, OBPropertyValue *value) = 0;
    virtual void getPropertyRange(uint64_t propertyId, OBPropertyRange *range) = 0;
};

class IPropertyExtensionPort {
public:
    virtual ~IPropertyExtensionPort() noexcept                                                          = default;
    virtual void                 setFirmwareData(uint64_t propertyId, const std::vector<uint8_t> &data) = 0;
    virtual std::vector<uint8_t> getFirmwareData(uint64_t propertyId)                                   = 0;
};

class IPropertyAccessor {
public:
    virtual ~IPropertyAccessor() noexcept = default;

    void registerProperty(uint32_t protectedId, OBPermissionType permission, std::shared_ptr<IPropertyPort> port);
    void aliasProperty(uint32_t aliasId, uint32_t protectedId);

    bool checkProperty(uint64_t propertyId, OBPermissionType permission) const;

    virtual void setPropertyValue(uint64_t propertyId, OBPropertyValue value)  = 0;
    virtual void getPropertyValue(uint64_t propertyId, OBPropertyValue *value) = 0;
    virtual void getPropertyRange(uint64_t propertyId, OBPropertyRange *range) = 0;

    virtual void                 setFirmwareData(uint64_t propertyId, const std::vector<uint8_t> &data) = 0;
    virtual std::vector<uint8_t> getFirmwareData(uint64_t propertyId)                                   = 0;

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value || std::is_same<T, bool>::value, void>::type setPropertyValueT(uint64_t propertyId, const T &value) {
        OBPropertyValue obValue;
        obValue.intValue = static_cast<int32_t>(value);
        setPropertyValue(propertyId, obValue);
    }

    template <typename T> typename std::enable_if<std::is_same<T, float>::value, T>::type setPropertyValueT(uint64_t propertyId) {
        OBPropertyValue obValue;
        obValue.floatValue = static_cast<float>(value);
        setPropertyValue(propertyId, obValue);
    }

    template <typename T> typename std::enable_if<std::is_integral<T>::value || std::is_same<T, bool>::value, T>::type getPropertyValueT(uint64_t propertyId) {
        OBPropertyValue obValue;
        getPropertyValue(propertyId, &obValue);
        return static_cast<T>(obValue.intValue);
    }

    template <typename T> typename std::enable_if<std::is_same<T, float>::value, T>::type getPropertyValueT(uint64_t propertyId) {
        OBPropertyValue obValue;
        getPropertyValue(propertyId, &obValue);
        return static_cast<T>(obValue.floatValue);
    }

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value || std::is_same<T, bool>::value, OBPropertyRangeT<T>>::type getPropertyRangeT(uint64_t propertyId) {
        OBPropertyRangeT<T> rangeT;
        OBPropertyRange     range;
        getPropertyRange(propertyId, (OBPropertyRange *)&range);

        rangeT.cur  = range.cur.intValue;
        rangeT.max  = range.max.intValue;
        rangeT.min  = range.min.intValue;
        rangeT.step = range.step.intValue;
        rangeT.def  = range.def.intValue;

        return rangeT;
    }

    template <typename T> typename std::enable_if<std::is_same<T, float>::value, OBPropertyRangeT<T>>::type getPropertyRangeT(uint64_t propertyId) {
        OBPropertyRangeT<T> rangeT;
        OBPropertyRange     range;
        getPropertyRange(propertyId, (OBPropertyRange *)&range);

        rangeT.cur  = range.cur.floatValue;
        rangeT.max  = range.max.floatValue;
        rangeT.min  = range.min.floatValue;
        rangeT.step = range.step.floatValue;
        rangeT.def  = range.def.floatValue;

        return rangeT;
    }

    template <typename T> void setFirmwareDataT(uint64_t propertyId, const T &data) {
        std::vector<uint8_t> vec(sizeof(T));
        std::memcpy(vec.data(), &data, sizeof(T));
        setFirmwareData(propertyId, vec);
    }

    template <typename T> T getFirmwareDataT(uint64_t propertyId) {
        std::vector<uint8_t> vec = getFirmwareData(propertyId);
        T                    data;
        if(vec.size() != sizeof(T)) {
            LOG_WARN("Firmware data size is not match with property type");
        }
        std::memcpy(&data, vec.data(), std::min(vec.size(), sizeof(T)));
        return data;
    }
};

}  // namespace libobsensor