#pragma once

#include "openobsdk/h/ObTypes.h"

namespace libobsensor{
namespace utils {

uint32_t getBytesPerPixel(OBFormat format);
uint32_t calcVideoFrameMaxDataSize(OBFormat format, uint32_t width, uint32_t height);

OBFrameType mapStreamTypeToFrameType(OBStreamType type);
OBStreamType mapFrameTypeToStreamType(OBFrameType type);

}
}