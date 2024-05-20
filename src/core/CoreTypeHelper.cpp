#include "CoreTypeHelper.hpp"
#include "exception/ObException.hpp"

namespace ob {
namespace type_helper {

uint32_t getBytesPerPixel(OBFormat format) {
    // bytes-per-pixel value
    uint32_t nBytesPerPixel = 0;
    switch(format) {
    case OB_FORMAT_Y8:
    case OB_FORMAT_BA81:
        nBytesPerPixel = sizeof(uint8_t);
        break;
    case OB_FORMAT_Y10:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y11:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y12:
    case OB_FORMAT_NV12:
    case OB_FORMAT_YV12:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y14:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_YUYV:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_BYR2:
    case OB_FORMAT_RW16:
    case OB_FORMAT_DISP16:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
        nBytesPerPixel = sizeof(uint8_t) * 3;
        break;
    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA:
        nBytesPerPixel = sizeof(uint8_t) * 4;
        break;
    case OB_FORMAT_POINT:
        nBytesPerPixel = sizeof(uint8_t) * 12;
        break;
    case OB_FORMAT_RGB_POINT:
        nBytesPerPixel = sizeof(uint8_t) * 24;
        break;
    default:
        throw invalid_value_exception("Unsupported image format or invalid encoding detected. Unable to determine the byte-per-pixel value.");
        break;
    }

    return nBytesPerPixel;
}

uint32_t calcVideoFrameMaxDataSize(OBFormat format, uint32_t width, uint32_t height) {
    uint32_t maxFrameDataSize = height * width * 3;
    switch(format) {
    case OB_FORMAT_NV21:
    case OB_FORMAT_I420:
        maxFrameDataSize = height * width * 3 / 2;
        break;
    case OB_FORMAT_MJPG:
    case OB_FORMAT_H264:
    case OB_FORMAT_H265:
    case OB_FORMAT_HEVC:
    case OB_FORMAT_Y8:
    case OB_FORMAT_BA81:
        maxFrameDataSize = height * width;
        break;
    case OB_FORMAT_YUYV:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_YUY2:
    case OB_FORMAT_Y16:
    case OB_FORMAT_BYR2:
    case OB_FORMAT_RW16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_RLE:
    case OB_FORMAT_RVL:
    case OB_FORMAT_DISP16:
        maxFrameDataSize = height * width * 2;
        break;
    case OB_FORMAT_Y10:
        maxFrameDataSize = (uint32_t)((float)height * width * 10.0f / 8.0f + 0.5);
        break;
    case OB_FORMAT_Y11:
        maxFrameDataSize = (uint32_t)((float)height * width * 11.0f / 8.0f + 0.5);
        break;
    case OB_FORMAT_Y12:
    case OB_FORMAT_NV12:
    case OB_FORMAT_YV12:
        maxFrameDataSize = (uint32_t)((float)height * width * 12.0f / 8.0f + 0.5);
        break;
    case OB_FORMAT_Y14:
        maxFrameDataSize = (uint32_t)((float)height * width * 14.0f / 8.0f + 0.5);
        break;
    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA:
        maxFrameDataSize = width * height * 4;
        break;
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
        maxFrameDataSize = width * height * 3;
        break;
    default:
        LOG_WARN("Unknown video frame format!");
        maxFrameDataSize = width * height * 3;
        break;
    }

    return maxFrameDataSize;
}

}  // namespace type_helper
}  // namespace ob