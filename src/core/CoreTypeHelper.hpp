#pragma once

#include "openobsdk/h/ObTypes.h"

namespace ob {
namespace type_helper {

uint32_t getBytesPerPixel(OBFormat format);
uint32_t calcVideoFrameMaxDataSize(OBFormat format, uint32_t width, uint32_t height);

}
}