// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include <map>
#include <vector>
#include <functional>

#if !defined(WIN32) && defined(BUILD_USB_PORT)
#include <libuvc/libuvc.h>
#endif
namespace libobsensor {
namespace pal {

struct VideoFrameObject {
    VideoFrameObject()
        : index(0),
          systemTime(0),
          deviceTime(0),
          format(OB_FORMAT_UNKNOWN),
          frameSize(0),
          frameData(nullptr),
          metadataSize(0),
          metadata(nullptr),
          scrDataBuf(nullptr),
          scrDataSize(0) {}

    void copyInfo(const VideoFrameObject source) {
        index        = source.index;
        systemTime   = source.systemTime;
        deviceTime   = source.deviceTime;
        format       = source.format;
        frameSize    = source.frameSize;
        frameData    = source.frameData;
        metadataSize = source.metadataSize;
        metadata     = source.metadata;
        scrDataBuf   = source.scrDataBuf;
        scrDataSize  = source.scrDataSize;
    }

    uint64_t index;
    uint64_t systemTime;
    uint64_t deviceTime;
    OBFormat format;
    uint32_t frameSize;
    void    *frameData;
    uint8_t  metadataSize;
    void    *metadata;
    void    *scrDataBuf;  // source clock reference, 通常是6个字节
    uint8_t  scrDataSize;
};

using VideoFrameCallback = std::function<void(const VideoFrameObject &)>;

#define LIBUVC_NUM_TRANSFER_LOW_FRAME_BUFS 20
#define LIBUVC_TRANSFER_LOW_FRAME_SIZE 10

#define LIBUVC_XFER_BUF_SIZE (16 * 1024 * 1024)
#define LIBUVC_XFER_META_BUF_SIZE (4 * 1024)

#pragma pack(push, 1)
struct StandardUvcFramePayloadHeader {
    uint8_t  bHeaderLength;
    uint8_t  bmHeaderInfo;
    uint32_t dwPresentationTime;
    uint8_t  scrSourceClock[6];
};
#pragma pack(pop)
#define UVC_PAYLOAD_HEADER_SRC_LENGTH (6)
#define UVC_PAYLOAD_HEADER_SRC_OFFSET (6)



template <typename T> uint32_t fourCc2Int(const T a, const T b, const T c, const T d) {
    static_assert((std::is_integral<T>::value), "fourcc supports integral built-in types only");
    return ((static_cast<uint32_t>(a) << 24) | (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(c) << 8) | (static_cast<uint32_t>(d) << 0));
}

#if !defined(WIN32) && defined(BUILD_USB_PORT)
const std::map<uint32_t, uvc_frame_format> fourccToUvcFormatMap = {
    { fourCc2Int('U', 'Y', 'V', 'Y'), UVC_FRAME_FORMAT_UYVY }, { fourCc2Int('Y', 'U', 'Y', '2'), UVC_FRAME_FORMAT_YUYV },
    { fourCc2Int('N', 'V', '1', '2'), UVC_FRAME_FORMAT_NV12 }, { fourCc2Int('I', '4', '2', '0'), UVC_FRAME_FORMAT_I420 },
    { fourCc2Int('N', 'V', '2', '1'), UVC_FRAME_FORMAT_NV21 }, { fourCc2Int('M', 'J', 'P', 'G'), UVC_FRAME_FORMAT_MJPEG },
    { fourCc2Int('H', '2', '6', '4'), UVC_FRAME_FORMAT_H264 }, { fourCc2Int('H', 'E', 'V', 'C'), UVC_FRAME_FORMAT_HEVC },
    { fourCc2Int('Y', '8', ' ', ' '), UVC_FRAME_FORMAT_Y8 },   { fourCc2Int('Y', '1', '0', ' '), UVC_FRAME_FORMAT_Y10 },
    { fourCc2Int('Y', '1', '1', ' '), UVC_FRAME_FORMAT_Y11 },  { fourCc2Int('Y', '1', '2', ' '), UVC_FRAME_FORMAT_Y12 },
    { fourCc2Int('Y', '1', '4', ' '), UVC_FRAME_FORMAT_Y14 },  { fourCc2Int('Y', '1', '6', ' '), UVC_FRAME_FORMAT_Y16 },
    { fourCc2Int('R', 'V', 'L', ' '), UVC_FRAME_FORMAT_RVL },  { fourCc2Int('Z', '1', '6', ' '), UVC_FRAME_FORMAT_Z16 },
    { fourCc2Int('Y', 'V', '1', '2'), UVC_FRAME_FORMAT_YV12 }, { fourCc2Int('B', 'A', '8', '1'), UVC_FRAME_FORMAT_BA81 },
};
#endif


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

template <typename T> T as(const std::vector<uint8_t> &data, size_t index) {
    T rv = 0;
    for(int i = 0; i < sizeof(T); i++) {
        rv += data[index + i] << (i * 8);
    }
    return rv;
}

#define XU_MAX_DATA_LENGTH 1024
typedef enum {
    OB_VENDOR_XU_CTRL_ID_512  = 1,
    OB_VENDOR_XU_CTRL_ID_64   = 2,
    OB_VENDOR_XU_CTRL_ID_1024 = 3,
} ObVendorXuCtrlId;

struct Guid {
    uint32_t data1;
    uint16_t data2, data3;
    uint8_t  data4[8];
};

// subdevice and node fields are assigned by Host driver; unit and GUID are
// hard-coded in camera firmware
struct ObExtensionUnit {
    // int     subdevice;
    uint8_t unit;
    // int     node;
    Guid id;
};
const ObExtensionUnit OB_COMMON_XU_UNIT = { 4, { 0xA55751A1, 0xF3C5, 0x4A5E, { 0x8D, 0x5A, 0x68, 0x54, 0xB8, 0xFA, 0x27, 0x16 } } };
const ObExtensionUnit OB_G2R_XU_UNIT    = { 3, { 0xC9606CCB, 0x594C, 0x4D25, { 0xaf, 0x47, 0xcc, 0xc4, 0x96, 0x43, 0x59, 0x95 } } };
// RGB - 松翰(jarvis)
// const pal::ObExtensionUnit OB_COMMON_VENDOR_XU = { 3, { 0x28F03370, 0x6311, 0x4A2E, { 0xBA, 0x2C, 0x68, 0x90, 0xEB, 0x33, 0x40, 0x16 } } };

enum PowerState { kD0, kD3 };
}  // namespace pal
}  // namespace libobsensor
