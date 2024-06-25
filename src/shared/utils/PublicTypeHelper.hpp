#pragma once

#include "openobsdk/h/ObTypes.h"
#include <iostream>

namespace libobsensor {
namespace utils {
float    getBytesPerPixel(OBFormat format);
uint32_t calcDefaultStrideBytes(OBFormat format, uint32_t width);
uint32_t calcVideoFrameMaxDataSize(OBFormat format, uint32_t width, uint32_t height);

OBFrameType  mapStreamTypeToFrameType(OBStreamType type);
OBStreamType mapFrameTypeToStreamType(OBFrameType type);
OBStreamType mapSensorTypeToStreamType(OBSensorType type);
OBSensorType mapStreamTypeToSensorType(OBStreamType type);

const std::string &getSensorName(OBSensorType type);
OBFormat     strToOBFormat(const std::string &str);

template <typename T> uint32_t fourCc2Int(const T a, const T b, const T c, const T d) {
    static_assert((std::is_integral<T>::value), "fourcc supports integral built-in types only");
    return ((static_cast<uint32_t>(a) << 24) | (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(c) << 8) | (static_cast<uint32_t>(d) << 0));
}
OBFormat uvcFourccToOBFormat(uint32_t fourcc);
uint32_t obFormatToUvcFourcc(OBFormat format);

float mapIMUSampleRateToValue(OBIMUSampleRate rate);

}  // namespace utils
}  // namespace libobsensor

std::ostream &operator<<(std::ostream &os, const OBFormat &format);
std::ostream &operator<<(std::ostream &os, const OBFrameType &type);
std::ostream &operator<<(std::ostream &os, const OBStreamType &type);
std::ostream &operator<<(std::ostream &os, const OBSensorType &type);
std::ostream &operator<<(std::ostream &os, const OBIMUSampleRate &rate);  // also for accel
std::ostream &operator<<(std::ostream &os, const OBGyroFullScaleRange &range);
std::ostream &operator<<(std::ostream &os, const OBAccelFullScaleRange &range);