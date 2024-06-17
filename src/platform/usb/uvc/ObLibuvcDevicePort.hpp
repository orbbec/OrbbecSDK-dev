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
#include "stream/StreamProfile.hpp"

namespace libobsensor {

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
    ~ObLibuvcDevicePort() noexcept override;

    StreamProfileListUnsafe getStreamProfileList() override;
    void                    startStream(std::shared_ptr<const StreamProfile> profile, FrameCallbackUnsafe callback) override;
    void                    stopStream(std::shared_ptr<const StreamProfile> profile) override;
    void                    stopAllStream() override;

    virtual bool sendData(const uint8_t *data, uint32_t dataLen);
    virtual bool recvData(uint8_t *data, uint32_t *dataLen);

     uint32_t sendAndReceive(const uint8_t *sendData, uint32_t sendLen, uint8_t *recvData, uint32_t exceptedRecvLen) override;

    bool            getPu(uint32_t propertyId, int32_t &value) override;
    bool            setPu(uint32_t propertyId, int32_t value) override;
    UvcControlRange getPuRange(uint32_t propertyId) override;

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

}  // namespace libobsensor