// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.
#pragma once
#include "openobsdk/h/ObTypes.h"

#include "UvcTypes.hpp"
#include "UvcDevicePort.hpp"
#include "usb/backend/Enumerator.hpp"
#include "usb/backend/Messenger.hpp"
#include "usb/backend/Device.hpp"

#include <cstdio>
#include <cstdlib>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <libuvc/libuvc.h>

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

class ObLibuvcDevicePort : public UvcDevicePort {
private:
    struct OBUvcStreamHandle {
        OBUvcStreamHandle(std::shared_ptr<const VideoStreamProfile> &profile_, std::function<void(std::shared_ptr<Frame>)> callback_,
                          uvc_stream_handle_t *streamHandle_)
            : profile(profile_), callback(std::move(callback_)), streamHandle(streamHandle_) {}
        std::shared_ptr<const VideoStreamProfile>   profile;
        std::function<void(std::shared_ptr<Frame>)> callback;
        uvc_stream_handle_t                        *streamHandle;
    };

    typedef struct {
        uint32_t width;
        uint32_t height;
        uint32_t fourcc;  // format fourcc code
        uint32_t fps;
        uint8_t  interfaceNumber;
        uint8_t  endpointAddress;
    } uvcProfile;

public:
    ObLibuvcDevicePort(std::shared_ptr<UsbDevice> usbDev, std::shared_ptr<const USBSourcePortInfo> portInfo);
    virtual ~ObLibuvcDevicePort() noexcept;

    std::vector<std::shared_ptr<const VideoStreamProfile>> getStreamProfileList() override;
    void startStream(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) override;
    void stopStream(std::shared_ptr<const VideoStreamProfile> profile) override;
    void stopAllStream() override;

    virtual bool sendData(const uint8_t *data, uint32_t dataLen);
    virtual bool recvData(uint8_t *data, uint32_t *dataLen);

    bool         getPu(OBPropertyID propertyId, int32_t &value) override;
    bool         setPu(OBPropertyID propertyId, int32_t value) override;
    ControlRange getPuRange(OBPropertyID propertyId) override;

    std::shared_ptr<const SourcePortInfo> getSourcePortInfo() const override;

private:
    virtual bool getXu(uint8_t ctrl, uint8_t *data, uint32_t *len);
    virtual bool setXu(uint8_t ctrl, const uint8_t *data, uint32_t len);

#ifdef __ANDROID__
    virtual std::string getUsbConnectType() override;
#endif

private:
    int32_t                 uvcCtrlValueTranslate(uvc_req_code action, OBPropertyID propertyId, int32_t value) const;
    static void             onFrameCallback(uvc_frame *frame, void *user_ptr);
    std::vector<uvcProfile> queryAvailableUvcProfile() const;

    int     obPropToUvcCS(OBPropertyID propertyId, int &unit) const;
    int32_t getCtrl(uvc_req_code action, uint8_t control, uint8_t unit) const;
    void    setCtrl(uvc_req_code action, uint8_t control, uint8_t unit, int32_t value) const;

private:
    std::recursive_mutex                     ctrlTransferMutex_;
    std::shared_ptr<UsbDevice>               usbDev_;
    uvc_device_t                            *uvcDev_;
    uvc_device_handle_t                     *devHandle_;
    libusb_device_handle                    *usbDevH_;
    uvc_context_t                           *uvcCtx_;
    std::shared_ptr<const USBSourcePortInfo> portInfo_;

    std::mutex                                      streamMutex_;
    std::vector<std::shared_ptr<OBUvcStreamHandle>> streamHandles_;
};
}  // namespace pal
}  // namespace libobsensor