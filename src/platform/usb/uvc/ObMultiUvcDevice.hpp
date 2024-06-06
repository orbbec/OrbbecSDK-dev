#pragma once

#include "UvcDevicePort.hpp"
#include "platform/usb/backend/Device.hpp"

namespace libobsensor {

namespace pal {

struct StreamFrameCallbackObject {
    bool               hasInit  = false;
    FrameCallbackUnsafe callback = nullptr;
};

class ObMultiUvcDevice : public UvcDevicePort {

public:
    ObMultiUvcDevice(std::shared_ptr<UsbDevice> usbDev, std::shared_ptr<const USBSourcePortInfo> portInfo);
    ~ObMultiUvcDevice() noexcept;

    virtual std::shared_ptr<const SourcePortInfo> getSourcePortInfo(void) const override;
    virtual std::vector<std::shared_ptr<const VideoStreamProfile>>             getStreamProfileList() override;
    virtual void                                  startStream(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) override;
    virtual void                                  stopStream(std::shared_ptr<const VideoStreamProfile> profile) override;
    virtual void                                  stopAllStream() override;

    virtual bool         getPu(OBPropertyID propertyId, int32_t &value) override;
    virtual bool         setPu(OBPropertyID propertyId, int32_t value) override;
    virtual ControlRange getPuRange(OBPropertyID propertyId) override;

    virtual bool sendData(const uint8_t *data, const uint32_t dataLen) override;
    virtual bool recvData(uint8_t *data, uint32_t *dataLen) override;

    virtual bool getStreamStarted();

    virtual void setTimestampOffsetFlag(bool flag) {
        timestampOffsetFlag_ = flag;
    }

#ifdef __ANDROID__
    virtual std::string getUsbConnectType() override;
#endif

private:
    void onMultiFrameRawDataCallback(pal::VideoFrameObject fo);

private:
    std::shared_ptr<pal::UvcDevicePort>          depthUvcPort_;
    std::shared_ptr<const VideoStreamProfile> profile_;
    std::vector<VideoFrameCallback>   frameCallbacks_;
    std::recursive_mutex                         multiStreamMutex_;
    bool                                         streamStart_;
    bool                                         timestampOffsetFlag_;
};

}  // namespace pal

}  // namespace libobsensor
