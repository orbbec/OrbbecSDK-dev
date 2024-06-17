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
const ObExtensionUnit OB_G330_XU_UNIT    = { 3, { 0xC9606CCB, 0x594C, 0x4D25, { 0xaf, 0x47, 0xcc, 0xc4, 0x96, 0x43, 0x59, 0x95 } } };
// RGB - 松翰(jarvis)
// const ObExtensionUnit OB_COMMON_VENDOR_XU = { 3, { 0x28F03370, 0x6311, 0x4A2E, { 0xBA, 0x2C, 0x68, 0x90, 0xEB, 0x33, 0x40, 0x16 } } };

enum PowerState { kD0, kD3 };

}  // namespace libobsensor
