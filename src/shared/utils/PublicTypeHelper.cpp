#include "PublicTypeHelper.hpp"
#include "exception/ObException.hpp"

#include <map>

namespace libobsensor {
namespace utils {

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

OBFrameType mapStreamTypeToFrameType(OBStreamType type) {
    switch(type) {
    case OB_STREAM_VIDEO:
        return OB_FRAME_VIDEO;
    case OB_STREAM_IR:
        return OB_FRAME_IR;
    case OB_STREAM_COLOR:
        return OB_FRAME_COLOR;
    case OB_STREAM_DEPTH:
        return OB_FRAME_DEPTH;
    case OB_STREAM_ACCEL:
        return OB_FRAME_ACCEL;
    case OB_STREAM_GYRO:
        return OB_FRAME_GYRO;
    case OB_STREAM_IR_LEFT:
        return OB_FRAME_IR_LEFT;
    case OB_STREAM_IR_RIGHT:
        return OB_FRAME_IR_RIGHT;
    case OB_STREAM_RAW_PHASE:
        return OB_FRAME_RAW_PHASE;
    case OB_STREAM_DISPARITY:
        return OB_FRAME_DISPARITY;
    default:
        break;
    }
    return OB_FRAME_UNKNOWN;
}

OBStreamType mapFrameTypeToStreamType(OBFrameType type) {
    switch(type) {
    case OB_FRAME_VIDEO:
        return OB_STREAM_VIDEO;
    case OB_FRAME_IR:
        return OB_STREAM_IR;
    case OB_FRAME_COLOR:
        return OB_STREAM_COLOR;
    case OB_FRAME_DEPTH:
        return OB_STREAM_DEPTH;
    case OB_FRAME_ACCEL:
        return OB_STREAM_ACCEL;
    case OB_FRAME_GYRO:
        return OB_STREAM_GYRO;
    case OB_FRAME_IR_LEFT:
        return OB_STREAM_IR_LEFT;
    case OB_FRAME_IR_RIGHT:
        return OB_STREAM_IR_RIGHT;
    case OB_FRAME_RAW_PHASE:
        return OB_STREAM_RAW_PHASE;
    case OB_FRAME_DISPARITY:
        return OB_STREAM_DISPARITY;
    default:
        break;
    }
    return OB_STREAM_UNKNOWN;
}

OBStreamType mapSensorTypeToStreamType(OBSensorType type) {
    switch(type) {
    case OB_SENSOR_IR:
        return OB_STREAM_IR;
    case OB_SENSOR_COLOR:
        return OB_STREAM_COLOR;
    case OB_SENSOR_DEPTH:
        return OB_STREAM_DEPTH;
    case OB_SENSOR_ACCEL:
        return OB_STREAM_ACCEL;
    case OB_SENSOR_GYRO:
        return OB_STREAM_GYRO;
    case OB_SENSOR_IR_LEFT:
        return OB_STREAM_IR_LEFT;
    case OB_SENSOR_IR_RIGHT:
        return OB_STREAM_IR_RIGHT;
    case OB_SENSOR_RAW_PHASE:
        return OB_STREAM_RAW_PHASE;
    default:
        break;
    }
    return OB_STREAM_UNKNOWN;
}

    const std::map<uint32_t, OBFormat> fourccToOBFormat = {
        { fourCc2Int('U', 'Y', 'V', 'Y'), OB_FORMAT_UYVY }, { fourCc2Int('Y', 'U', 'Y', '2'), OB_FORMAT_YUYV },
        { fourCc2Int('Y', 'U', 'Y', 'V'), OB_FORMAT_YUYV }, { fourCc2Int('N', 'V', '1', '2'), OB_FORMAT_NV12 },
        { fourCc2Int('N', 'V', '2', '1'), OB_FORMAT_NV21 }, { fourCc2Int('M', 'J', 'P', 'G'), OB_FORMAT_MJPG },
        { fourCc2Int('H', '2', '6', '4'), OB_FORMAT_H264 }, { fourCc2Int('H', '2', '6', '5'), OB_FORMAT_H265 },
        { fourCc2Int('Y', '1', '2', ' '), OB_FORMAT_Y12 },  { fourCc2Int('Y', '1', '6', ' '), OB_FORMAT_Y16 },
        { fourCc2Int('G', 'R', 'A', 'Y'), OB_FORMAT_GRAY }, { fourCc2Int('Y', '1', '1', ' '), OB_FORMAT_Y11 },
        { fourCc2Int('Y', '8', ' ', ' '), OB_FORMAT_Y8 },   { fourCc2Int('Y', '1', '0', ' '), OB_FORMAT_Y10 },
        { fourCc2Int('H', 'E', 'V', 'C'), OB_FORMAT_HEVC }, { fourCc2Int('Y', '1', '4', ' '), OB_FORMAT_Y14 },
        { fourCc2Int('I', '4', '2', '0'), OB_FORMAT_I420 }, { fourCc2Int('Z', '1', '6', ' '), OB_FORMAT_Z16 },
        { fourCc2Int('Y', 'V', '1', '2'), OB_FORMAT_YV12 }, { fourCc2Int('B', 'A', '8', '1'), OB_FORMAT_BA81 },
        { fourCc2Int('B', 'Y', 'R', '2'), OB_FORMAT_BYR2 }, { fourCc2Int('R', 'W', '1', '6'), OB_FORMAT_RW16 },
    };

OBFormat uvcFourccToOBFormat(uint32_t fourcc) {
    auto it = fourccToOBFormat.find(fourcc);
    if(it != fourccToOBFormat.end()) {
        return it->second;
    }
    return OB_FORMAT_UNKNOWN;
}

uint32_t obFormatToUvcFourcc(OBFormat format) {
    auto it = std::find_if(fourccToOBFormat.begin(), fourccToOBFormat.end(), [format](const std::pair<uint32_t, OBFormat>& p) {
        return p.second == format;
    });
    if(it != fourccToOBFormat.end()) {
        return it->first;
    }
    return 0;
}

}  // namespace utils
}  // namespace libobsensor