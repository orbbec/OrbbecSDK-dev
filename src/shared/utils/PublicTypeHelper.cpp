#include "PublicTypeHelper.hpp"
#include "exception/ObException.hpp"

#include <map>

namespace libobsensor {
namespace utils {

float getBytesPerPixel(OBFormat format) {
    float bytesPerPixel = 0.f;
    switch(format) {
    case OB_FORMAT_Y8:
    case OB_FORMAT_BA81:
        bytesPerPixel = 1.f;
        break;
    case OB_FORMAT_Y10:
        bytesPerPixel = 10.f / 8.f;
        break;
    case OB_FORMAT_Y11:
        bytesPerPixel = 11.f / 8.f;
        break;
    case OB_FORMAT_Y12:
    case OB_FORMAT_NV12:
    case OB_FORMAT_YV12:
        bytesPerPixel = 12.f / 8.f;
        break;
    case OB_FORMAT_Y14:
        bytesPerPixel = 14.f / 8.f;
        break;
    case OB_FORMAT_NV21:
    case OB_FORMAT_I420:
        bytesPerPixel = 1.5f;
        break;
    case OB_FORMAT_Y16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_YUYV:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_BYR2:
    case OB_FORMAT_RW16:
    case OB_FORMAT_DISP16:
        bytesPerPixel = 2.f;
        break;
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
        bytesPerPixel = 3.f;
        break;
    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA:
        bytesPerPixel = 4.f;
        break;
    case OB_FORMAT_POINT:
        bytesPerPixel = 12.f;
        break;
    case OB_FORMAT_RGB_POINT:
        bytesPerPixel = 24.f;
        break;
    default:
        throw invalid_value_exception("Unsupported image format or invalid encoding detected. Unable to determine the byte-per-pixel value.");
        break;
    }

    return bytesPerPixel;
}

uint32_t calcDefaultStrideBytes(OBFormat format, uint32_t width) {
    float    bytesPerPixel = getBytesPerPixel(format);
    uint32_t stride        = (uint32_t)(width * bytesPerPixel + 0.5);
    return stride;
}

uint32_t calcVideoFrameMaxDataSize(OBFormat format, uint32_t width, uint32_t height) {
    uint32_t maxFrameDataSize = height * width * 3;
    switch(format) {
    case OB_FORMAT_MJPG:
    case OB_FORMAT_H264:
    case OB_FORMAT_H265:
    case OB_FORMAT_HEVC:
        maxFrameDataSize = height * width;
        break;
    case OB_FORMAT_RLE:
    case OB_FORMAT_RVL:
        maxFrameDataSize = height * width * 2;
        break;
    default:  // assume planar format
        maxFrameDataSize = height * calcDefaultStrideBytes(format, width);
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

OBSensorType mapStreamTypeToSensorType(OBStreamType type) {
    switch(type) {
    case OB_STREAM_IR:
        return OB_SENSOR_IR;
    case OB_STREAM_COLOR:
        return OB_SENSOR_COLOR;
    case OB_STREAM_DEPTH:
        return OB_SENSOR_DEPTH;
    case OB_STREAM_ACCEL:
        return OB_SENSOR_ACCEL;
    case OB_STREAM_GYRO:
        return OB_SENSOR_GYRO;
    case OB_STREAM_IR_LEFT:
        return OB_SENSOR_IR_LEFT;
    case OB_STREAM_IR_RIGHT:
        return OB_SENSOR_IR_RIGHT;
    case OB_STREAM_RAW_PHASE:
        return OB_SENSOR_RAW_PHASE;
    default:
        break;
    }
    return OB_SENSOR_UNKNOWN;
}

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

OBFormat uvcFourccToOBFormat(uint32_t fourcc) {
    auto it = fourccToOBFormat.find(fourcc);
    if(it != fourccToOBFormat.end()) {
        return it->second;
    }
    return OB_FORMAT_UNKNOWN;
}

uint32_t obFormatToUvcFourcc(OBFormat format) {
    auto it = std::find_if(fourccToOBFormat.begin(), fourccToOBFormat.end(), [format](const std::pair<uint32_t, OBFormat> &p) { return p.second == format; });
    if(it != fourccToOBFormat.end()) {
        return it->first;
    }
    return 0;
}

float mapIMUSampleRateToValue(OBIMUSampleRate rate) {
    switch(rate) {
    case OB_SAMPLE_RATE_1_5625_HZ:
        return 1.5625f;
    case OB_SAMPLE_RATE_3_125_HZ:
        return 3.125f;
    case OB_SAMPLE_RATE_6_25_HZ:
        return 6.25f;
    case OB_SAMPLE_RATE_12_5_HZ:
        return 12.5f;
    case OB_SAMPLE_RATE_25_HZ:
        return 25.f;
    case OB_SAMPLE_RATE_50_HZ:
        return 50.f;
    case OB_SAMPLE_RATE_100_HZ:
        return 100.f;
    case OB_SAMPLE_RATE_200_HZ:
        return 200.f;
    case OB_SAMPLE_RATE_500_HZ:
        return 500.f;
    case OB_SAMPLE_RATE_1_KHZ:
        return 1000.f;
    case OB_SAMPLE_RATE_2_KHZ:
        return 2000.f;
    case OB_SAMPLE_RATE_4_KHZ:
        return 4000.f;
    case OB_SAMPLE_RATE_8_KHZ:
        return 8000.f;
    case OB_SAMPLE_RATE_16_KHZ:
        return 16000.f;
    case OB_SAMPLE_RATE_32_KHZ:
        return 32000.f;
    default:
        return 0.f;
    }
}

}  // namespace utils
}  // namespace libobsensor

std::ostream &operator<<(std::ostream &os, const OBFormat &format) {
    switch(format) {
    case OB_FORMAT_YUYV:
        os << "YUYV";
        break;
    case OB_FORMAT_YUY2:
        os << "YUY2";
        break;
    case OB_FORMAT_UYVY:
        os << "UYVY";
        break;
    case OB_FORMAT_NV12:
        os << "NV12";
        break;
    case OB_FORMAT_NV21:
        os << "NV21";
        break;
    case OB_FORMAT_MJPG:
        os << "MJPG";
        break;
    case OB_FORMAT_H264:
        os << "H264";
        break;
    case OB_FORMAT_H265:
        os << "H265";
        break;
    case OB_FORMAT_Y16:
        os << "Y16";
        break;
    case OB_FORMAT_Y8:
        os << "Y8";
        break;
    case OB_FORMAT_Y10:
        os << "Y10";
        break;
    case OB_FORMAT_Y11:
        os << "Y11";
        break;
    case OB_FORMAT_Y12:
        os << "Y12";
        break;
    case OB_FORMAT_GRAY:
        os << "GRAY";
        break;
    case OB_FORMAT_HEVC:
        os << "HEVC";
        break;
    case OB_FORMAT_I420:
        os << "I420";
        break;
    case OB_FORMAT_ACCEL:
        os << "ACCEL";
        break;
    case OB_FORMAT_GYRO:
        os << "GYRO";
        break;
    case OB_FORMAT_POINT:
        os << "POINT";
        break;
    case OB_FORMAT_RGB_POINT:
        os << "RGB_POINT";
        break;
    case OB_FORMAT_RLE:
        os << "RLE";
        break;
    case OB_FORMAT_RGB:
        os << "RGB";
        break;
    case OB_FORMAT_BGR:
        os << "BGR";
        break;
    case OB_FORMAT_Y14:
        os << "Y14";
        break;
    case OB_FORMAT_BGRA:
        os << "BGRA";
        break;
    case OB_FORMAT_COMPRESSED:
        os << "COMPRESSED";
        break;
    case OB_FORMAT_RVL:
        os << "RVL";
        break;
    case OB_FORMAT_Z16:
        os << "Z16";
        break;
    case OB_FORMAT_YV12:
        os << "YV12";
        break;
    case OB_FORMAT_BA81:
        os << "BA81";
        break;
    case OB_FORMAT_RGBA:
        os << "RGBA";
        break;
    case OB_FORMAT_BYR2:
        os << "BYR2";
        break;
    case OB_FORMAT_RW16:
        os << "RW16";
        break;
    case OB_FORMAT_DISP16:
        os << "DISP16";
        break;
    default:
        os << "Unknown format";
        break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OBFrameType &type) {
    switch(type) {
    case OB_FRAME_VIDEO:
        os << "FRAME_VIDEO";
        break;
    case OB_FRAME_IR:
        os << "FRAME_IR";
        break;
    case OB_FRAME_COLOR:
        os << "FRAME_COLOR";
        break;
    case OB_FRAME_DEPTH:
        os << "FRAME_DEPTH";
        break;
    case OB_FRAME_ACCEL:
        os << "FRAME_ACCEL";
        break;
    case OB_FRAME_GYRO:
        os << "FRAME_GYRO";
        break;
    case OB_FRAME_IR_LEFT:
        os << "FRAME_IR_LEFT";
        break;
    case OB_FRAME_IR_RIGHT:
        os << "FRAME_IR_RIGHT";
        break;
    case OB_FRAME_RAW_PHASE:
        os << "FRAME_RAW_PHASE";
        break;
    default:
        os << "Unknown frame type";
        break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OBStreamType &type) {
    switch(type) {
    case OB_STREAM_VIDEO:
        os << "STREAM_VIDEO";
        break;
    case OB_STREAM_IR:
        os << "STREAM_IR";
        break;
    case OB_STREAM_COLOR:
        os << "STREAM_COLOR";
        break;
    case OB_STREAM_DEPTH:
        os << "STREAM_DEPTH";
        break;
    case OB_STREAM_ACCEL:
        os << "STREAM_ACCEL";
        break;
    case OB_STREAM_GYRO:
        os << "STREAM_GYRO";
        break;
    case OB_STREAM_IR_LEFT:
        os << "STREAM_IR_LEFT";
        break;
    case OB_STREAM_IR_RIGHT:
        os << "STREAM_IR_RIGHT";
        break;
    case OB_STREAM_RAW_PHASE:
        os << "STREAM_RAW_PHASE";
        break;
    default:
        os << "Unknown stream type";
        break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OBSensorType &type) {
    switch(type) {
    case OB_SENSOR_IR:
        os << "SENSOR_IR";
        break;
    case OB_SENSOR_COLOR:
        os << "SENSOR_COLOR";
        break;
    case OB_SENSOR_DEPTH:
        os << "SENSOR_DEPTH";
        break;
    case OB_SENSOR_ACCEL:
        os << "SENSOR_ACCEL";
        break;
    case OB_SENSOR_GYRO:
        os << "SENSOR_GYRO";
        break;
    case OB_SENSOR_IR_LEFT:
        os << "SENSOR_IR_LEFT";
        break;
    case OB_SENSOR_IR_RIGHT:
        os << "SENSOR_IR_RIGHT";
        break;
    case OB_SENSOR_RAW_PHASE:
        os << "SENSOR_RAW_PHASE";
        break;
    default:
        os << "Unknown sensor type";
        break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OBIMUSampleRate &rate) {
    switch(rate) {
    case OB_SAMPLE_RATE_1_5625_HZ:
        os << "1.5625Hz";
        break;
    case OB_SAMPLE_RATE_3_125_HZ:
        os << "3.125Hz";
        break;
    case OB_SAMPLE_RATE_6_25_HZ:
        os << "6.25Hz";
        break;
    case OB_SAMPLE_RATE_12_5_HZ:
        os << "12.5Hz";
        break;
    case OB_SAMPLE_RATE_25_HZ:
        os << "25Hz";
        break;
    case OB_SAMPLE_RATE_50_HZ:
        os << "50Hz";
        break;
    case OB_SAMPLE_RATE_100_HZ:
        os << "100Hz";
        break;
    case OB_SAMPLE_RATE_200_HZ:
        os << "200Hz";
        break;
    case OB_SAMPLE_RATE_500_HZ:
        os << "500Hz";
        break;
    case OB_SAMPLE_RATE_1_KHZ:
        os << "1KHz";
        break;
    case OB_SAMPLE_RATE_2_KHZ:
        os << "2KHz";
        break;
    case OB_SAMPLE_RATE_4_KHZ:
        os << "4KHz";
        break;
    case OB_SAMPLE_RATE_8_KHZ:
        os << "8KHz";
        break;
    case OB_SAMPLE_RATE_16_KHZ:
        os << "16KHz";
        break;
    case OB_SAMPLE_RATE_32_KHZ:
        os << "32KHz";
        break;
    default:
        os << "Unknown IMU sample rate";
        break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OBGyroFullScaleRange &range) {
    switch(range) {
    case OB_GYRO_FS_16dps:
        os << "16dps";
        break;
    case OB_GYRO_FS_31dps:
        os << "31dps";
        break;
    case OB_GYRO_FS_62dps:
        os << "62dps";
        break;
    case OB_GYRO_FS_125dps:
        os << "125dps";
        break;
    case OB_GYRO_FS_250dps:
        os << "250dps";
        break;
    case OB_GYRO_FS_500dps:
        os << "500dps";
        break;
    case OB_GYRO_FS_1000dps:
        os << "1000dps";
        break;
    case OB_GYRO_FS_2000dps:
        os << "2000dps";
        break;
    default:
        os << "Unknown gyro full scale range";
        break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OBAccelFullScaleRange &params) {
    switch(params) {
    case OB_ACCEL_FS_2g:
        os << "2g";
        break;
    case OB_ACCEL_FS_4g:
        os << "4g";
        break;
    case OB_ACCEL_FS_8g:
        os << "8g";
        break;
    case OB_ACCEL_FS_16g:
        os << "16g";
        break;
    default:
        os << "Unknown accel full scale range";
        break;
    }
    return os;
}