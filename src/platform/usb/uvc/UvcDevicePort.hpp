#pragma once

#include "ISourcePort.hpp"
#include "UvcTypes.hpp"
#include "usb/backend/UsbTypes.hpp"
#include "openobsdk/h/Property.h"

namespace libobsensor {
namespace pal {


#undef min
#undef max
// data struct
struct ControlRange {
    ControlRange() = default;

    ControlRange(int32_t in_min, int32_t in_max, int32_t in_step, int32_t in_def) {
        populate_raw_data(min, in_min);
        populate_raw_data(max, in_max);
        populate_raw_data(step, in_step);
        populate_raw_data(def, in_def);
    }
    ControlRange(std::vector<uint8_t> in_min, std::vector<uint8_t> in_max, std::vector<uint8_t> in_step, std::vector<uint8_t> in_def)
        : min(std::move(in_min)), max(std::move(in_max)), step(std::move(in_step)), def(std::move(in_def)) {}

    std::vector<uint8_t> min;
    std::vector<uint8_t> max;
    std::vector<uint8_t> step;
    std::vector<uint8_t> def;

private:
    static void populate_raw_data(std::vector<uint8_t> &vec, int32_t value) {
        vec.resize(sizeof(value));
        auto data = reinterpret_cast<const uint8_t *>(&value);
        std::copy(data, data + sizeof(value), vec.data());
    }
};

class UvcDevicePort : public IVideoStreamPort, public IVendorDataPort {
public:
    virtual bool         getPu(OBPropertyID propertyId, int32_t &value) = 0;
    virtual bool         setPu(OBPropertyID propertyId, int32_t value)  = 0;
    virtual ControlRange getPuRange(OBPropertyID propertyId)            = 0;

    ~UvcDevicePort() noexcept override = default;

    virtual void updateXuUnit(const ObExtensionUnit &xuUnit) {
        xuUnit_ = xuUnit;
    };

#ifdef __ANDROID__
    virtual std::string getUsbConnectType() = 0;
#endif

protected:
    ObExtensionUnit xuUnit_ = OB_COMMON_XU_UNIT;
};

}  // namespace pal
}  // namespace libobsensor
