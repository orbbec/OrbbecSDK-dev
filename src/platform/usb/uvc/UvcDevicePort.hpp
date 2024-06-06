#pragma once

#include "ISourcePort.hpp"
#include "UvcTypes.hpp"
#include "usb/backend/UsbTypes.hpp"
#include "openobsdk/h/Property.h"

namespace libobsensor {
namespace pal {

const std::map<uint32_t, OBFormat> fourccToOBFormat = {
    { fourCc2Int('U', 'Y', 'V', 'Y'), OB_FORMAT_UYVY }, { fourCc2Int('Y', 'U', 'Y', '2'), OB_FORMAT_YUYV }, { fourCc2Int('Y', 'U', 'Y', 'V'), OB_FORMAT_YUYV },
    { fourCc2Int('N', 'V', '1', '2'), OB_FORMAT_NV12 }, { fourCc2Int('N', 'V', '2', '1'), OB_FORMAT_NV21 }, { fourCc2Int('M', 'J', 'P', 'G'), OB_FORMAT_MJPG },
    { fourCc2Int('H', '2', '6', '4'), OB_FORMAT_H264 }, { fourCc2Int('H', '2', '6', '5'), OB_FORMAT_H265 }, { fourCc2Int('Y', '1', '2', ' '), OB_FORMAT_Y12 },
    { fourCc2Int('Y', '1', '6', ' '), OB_FORMAT_Y16 },  { fourCc2Int('G', 'R', 'A', 'Y'), OB_FORMAT_GRAY }, { fourCc2Int('Y', '1', '1', ' '), OB_FORMAT_Y11 },
    { fourCc2Int('Y', '8', ' ', ' '), OB_FORMAT_Y8 },   { fourCc2Int('Y', '1', '0', ' '), OB_FORMAT_Y10 },  { fourCc2Int('H', 'E', 'V', 'C'), OB_FORMAT_HEVC },
    { fourCc2Int('Y', '1', '4', ' '), OB_FORMAT_Y14 },  { fourCc2Int('I', '4', '2', '0'), OB_FORMAT_I420 }, { fourCc2Int('Z', '1', '6', ' '), OB_FORMAT_Z16 },
    { fourCc2Int('Y', 'V', '1', '2'), OB_FORMAT_YV12 }, { fourCc2Int('B', 'A', '8', '1'), OB_FORMAT_BA81 }, { fourCc2Int('B', 'Y', 'R', '2'), OB_FORMAT_BYR2 },
    { fourCc2Int('R', 'W', '1', '6'), OB_FORMAT_RW16 },
};

// data struct
struct ControlRange {
    ControlRange() {}

    ControlRange(int32_t in_min, int32_t in_max, int32_t in_step, int32_t in_def) {
        populate_raw_data(min, in_min);
        populate_raw_data(max, in_max);
        populate_raw_data(step, in_step);
        populate_raw_data(def, in_def);
    }
    ControlRange(std::vector<uint8_t> in_min, std::vector<uint8_t> in_max, std::vector<uint8_t> in_step, std::vector<uint8_t> in_def) {
        min  = in_min;
        max  = in_max;
        step = in_step;
        def  = in_def;
    }
    std::vector<uint8_t> min;
    std::vector<uint8_t> max;
    std::vector<uint8_t> step;
    std::vector<uint8_t> def;

private:
    void populate_raw_data(std::vector<uint8_t> &vec, int32_t value) {
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
