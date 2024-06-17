#pragma once
#include <memory>
#include <vector>
#include <string>

#include "openobsdk/h/ObTypes.h"
#include "IFrame.hpp"
#include "IStreamProfile.hpp"

namespace libobsensor {
enum SourcePortType {
    SOURCE_PORT_USB_VENDOR = 0x00,
    SOURCE_PORT_USB_UVC,
    SOURCE_PORT_USB_MULTI_UVC,
    SOURCE_PORT_USB_HID,
    SOURCE_PORT_NET_VENDOR = 0x10,
    SOURCE_PORT_NET_VENDOR_STREAM,
    SOURCE_PORT_NET_RTSP,
    SOURCE_PORT_IPC_VENDOR,  // Inter-process communication port
    SOURCE_PORT_UNKNOWN = 0xff,
};

struct SourcePortInfo {
    virtual ~SourcePortInfo() noexcept = default;
    SourcePortType portType;
    virtual bool   equal(std::shared_ptr<const SourcePortInfo> cmpInfo) const = 0;
};

struct NetSourcePortInfo : public SourcePortInfo {
    ~NetSourcePortInfo() noexcept override = default;
    std::string address;
    uint16_t    port;
    std::string mac;
    std::string serialNumber;
    uint32_t    pid;
    bool        equal(std::shared_ptr<const SourcePortInfo> cmpInfo) const override {
        if(cmpInfo->portType != portType) {
            return false;
        }
        auto netCmpInfo = std::dynamic_pointer_cast<const NetSourcePortInfo>(cmpInfo);
        return (address == netCmpInfo->address) && (port == netCmpInfo->port) && (mac == netCmpInfo->mac) && (serialNumber == netCmpInfo->serialNumber)
               && (pid == netCmpInfo->pid);
    }
};

struct ShmStreamPortInfo : public SourcePortInfo {  // shared memory stream port
    ~ShmStreamPortInfo() noexcept override = default;
    std::string  shmName;
    int32_t      blockSize;
    int32_t      blockCount;
    virtual bool equal(std::shared_ptr<const SourcePortInfo> cmpInfo) const override {
        if(cmpInfo->portType != portType) {
            return false;
        }
        auto netCmpInfo = std::dynamic_pointer_cast<const ShmStreamPortInfo>(cmpInfo);
        return (shmName == netCmpInfo->shmName) && (blockSize == netCmpInfo->blockSize) && (blockCount == netCmpInfo->blockCount);
    };
};

struct USBSourcePortInfo : public SourcePortInfo {
    ~USBSourcePortInfo() noexcept override = default;
    USBSourcePortInfo() {}
    USBSourcePortInfo(SourcePortType type) {
        portType = type;
    }
    std::string url;  // usb device url
    std::string uid;
    uint16_t    vid = 0;  // usb device vid
    uint16_t    pid = 0;  // usb device pid
    std::string serial;   // usb device serial number
    std::string connSpec;

    std::string infUrl;        // interface url(interface uid)
    uint8_t     infIndex = 0;  // interface index
    std::string infName;       // interface name
    std::string hubId;         // hub id
    bool        equal(std::shared_ptr<const SourcePortInfo> cmpInfo) const override {
        if(cmpInfo->portType != portType) {
            return false;
        }
        auto netCmpInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(cmpInfo);
        return (url == netCmpInfo->url) && (vid == netCmpInfo->vid) && (pid == netCmpInfo->pid) && (infUrl == netCmpInfo->infUrl)
               && (infIndex == netCmpInfo->infIndex) && (infName == netCmpInfo->infName) && (hubId == netCmpInfo->hubId);
    };
};

typedef std::vector<std::shared_ptr<const SourcePortInfo>> SourcePortInfoList;

class ISourcePort {
public:
    virtual ~ISourcePort() noexcept = default;

    virtual std::shared_ptr<const SourcePortInfo> getSourcePortInfo() const = 0;
};

// for vendor command
class IVendorDataPort : virtual public ISourcePort {  // Virtual inheritance solves diamond inheritance problem
public:
    ~IVendorDataPort() noexcept override = default;

    virtual uint32_t sendAndReceive(const uint8_t *sendData, uint32_t sendLen, uint8_t *recvData, uint32_t exceptedRecvLen) = 0;
};

// for imu data stream
class IDataStreamPort : virtual public ISourcePort {  // Virtual inheritance solves diamond inheritance problem
public:
    ~IDataStreamPort() noexcept override = default;

    virtual void startStream(FrameCallbackUnsafe callback) = 0;
    virtual void stopStream()                              = 0;
};

// for video data stream: depth, color, ir, etc.
class IVideoStreamPort : virtual public ISourcePort {  // Virtual inheritance solves diamond inheritance problem
public:
    ~IVideoStreamPort() noexcept override = default;

    virtual StreamProfileListUnsafe getStreamProfileList()                                                                  = 0;
    virtual void                    startStream(std::shared_ptr<const StreamProfile> profile, FrameCallbackUnsafe callback) = 0;
    virtual void                    stopStream(std::shared_ptr<const StreamProfile> profile)                                = 0;
    virtual void                    stopAllStream()                                                                         = 0;
};
}  // namespace libobsensor
