#pragma once
#include "platform/usb/uvc/UvcDevicePort.hpp"
#include "platform/usb/backend/Device.hpp"

namespace libobsensor {
namespace pal {

class RawPhaseConverterDevice : public UvcDevicePort {

public:
    RawPhaseConverterDevice(std::shared_ptr<const USBSourcePortInfo> portInfo);
    virtual ~RawPhaseConverterDevice() noexcept;

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

    virtual std::vector<std::shared_ptr<const VideoStreamProfile>> getRawPhaseStreamProfileList();

    virtual bool getStreamStarted();

#ifdef __ANDROID__
    virtual std::string getUsbConnectType() override;
#endif

protected:
    void onFrameRawDataCallback(VideoFrameObject fo);

    // 需要实现处理函数,TODO：需要单独开启线程处理
    virtual void processFrame(VideoFrameObject fo) = 0;

protected:
    std::shared_ptr<pal::UvcDevicePort>          srcUvcPort_;
    std::shared_ptr<const VideoStreamProfile> profile_;
    std::map<OBStreamType, VideoFrameCallback>   frameCallbacks_;  // <OBStreamType, VideoFrameCallback>
    std::recursive_mutex                         streamMutex_;
    bool                                         streamStart_;
};

}  // namespace pal

}  // namespace libobsensor
