// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once
#include "WinHelpers.hpp"

#include "openobsdk/h/ObTypes.h"
#include "ObPal.hpp"
#include "usb/backend/Enumerator.hpp"
#include "usb/backend/UsbTypes.hpp"
#include "UvcDevicePort.hpp"

#include <mfapi.h>
#include <Ks.h>
#include <atlcomcli.h>
#include <atomic>
#include <ksproxy.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mutex>
#include <strmif.h>
#include <unordered_map>
#include <vidcap.h>
#include <wrl.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

static const std::vector<std::vector<std::pair<GUID, GUID>>> attributes_params = {
    { { MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID },
      { MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_CATEGORY, KSCATEGORY_SENSOR_CAMERA } },
    { { MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID } },
};

#pragma pack(push, 1)
struct UvcHeader {
    uint8_t length;  // UVC Metadata total length is
                     // limited by design to 255 bytes
    uint8_t  info;
    uint32_t timeStamp;
    uint8_t  reserve[2];
};
#pragma pack(pop)
constexpr uint8_t uvcHeaderSize = sizeof(UvcHeader);
namespace libobsensor {


class WinPal;

struct StreamObject {
    bool                                         streaming = false;
    manual_reset_event                           isFlushed;
    manual_reset_event                           hasStarted;
    std::shared_ptr<const VideoStreamProfile> profile = nullptr;
    uint32_t                                     frameCounter;
    FrameCallbackUnsafe                           callback = nullptr;
};

typedef std::function<void(const UsbDeviceInfo &, IMFActivate *)> USBDeviceInfoEnumCallback;

typedef struct FrameRate {
    unsigned int denominator;
    unsigned int numerator;
} FrameRate;

typedef struct MFProfile {
    uint32_t                               index;
    FrameRate                              min_rate;
    FrameRate                              max_rate;
    std::shared_ptr<VideoStreamProfile> profile;
} MFProfile;

template <class T> static void safe_release(T &ppT) {
    if(ppT) {
        ppT.Release();
        ppT = NULL;
    }
}

class WmfUvcDevicePort : public std::enable_shared_from_this<WmfUvcDevicePort>,
                     public UvcDevicePort,
                     public Microsoft::WRL::RuntimeClass<Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::ClassicCom>, IMFSourceReaderCallback> {
public:
    explicit WmfUvcDevicePort(std::shared_ptr<const USBSourcePortInfo> portInfo);
    ~WmfUvcDevicePort() noexcept override;
    std::shared_ptr<const SourcePortInfo> getSourcePortInfo(void) const override;
    StreamProfileListUnsafe            getStreamProfileList() override;
    void                                  startStream(std::shared_ptr<const StreamProfile> profile, FrameCallbackUnsafe callback) override;
    void                                  stopStream(std::shared_ptr<const StreamProfile> profile) override;
    void                                  stopAllStream() override;

    bool         getPu(uint64_t propertyId, int32_t &value) override;
    bool         setPu(uint64_t propertyId, int32_t value) override;
    UvcControlRange getPuRange(uint64_t propertyId) override;

    std::vector<uint8_t> sendAndReceive(const std::vector<uint8_t> &sendData, uint32_t exceptedRevLen) override;

    static bool isConnected(std::shared_ptr<const USBSourcePortInfo> info);
    static void foreachUvcDevice(const USBDeviceInfoEnumCallback& action);

private:
    IAMVideoProcAmp  *getVideoProc() const;
    IAMCameraControl *getCameraControl() const;

    void initXu();
    bool getXu(uint8_t ctrl, uint8_t *data, uint32_t *len);
    bool setXu(uint8_t ctrl, const uint8_t *data, uint32_t len);

private:
    friend class SourceReaderCallback;

    void                   playProfile(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback);
    void                   flush(int index);
    void                   checkConnection() const;
    CComPtr<IMFAttributes> createDeviceAttrs();
    CComPtr<IMFAttributes> createReaderAttrs();
    void                   foreachProfile(std::function<void(const MFProfile &profile, CComPtr<IMFMediaType> media_type, bool &quit)> action) const;

    void setPowerStateD0();  // create device source and source reader
    void setPowerStateD3();  // release device source and source reader
    void reloadSourceAndReader();

    void queryXuNodeId(const CComPtr<IKsTopologyInfo>& topologyInfo);

    void initTimeoutThread();
    void terminateTimeoutThread();
    void addTimeoutBarrier(std::string msg);
    void removeTimeoutBarrier();

private:
    std::recursive_mutex deviceMutex_;

    std::shared_ptr<const USBSourcePortInfo> portInfo_;
    PowerState                               powerState_ = kD3;

    CComPtr<IMFMediaSource>                 deviceSource_ = nullptr;
    CComPtr<IMFAttributes>                  deviceAttrs_  = nullptr;
    Microsoft::WRL::ComPtr<IMFSourceReader> streamReader_;
    CComPtr<IMFAttributes>                  readerAttrs_ = nullptr;

    CComPtr<IAMCameraControl> cameraControl_ = nullptr;
    CComPtr<IAMVideoProcAmp>  videoProc_     = nullptr;
    CComPtr<IKsControl>       xuKsControl_;
    int                       xuNodeId_;

    HRESULT readSampleResult_ = S_OK;

    std::map<uint32_t, StreamObject> streams_;  // <index, StreamObj>
    std::mutex                       streamsMutex_;

    std::atomic<bool> isStarted_ = {false};
    std::wstring      deviceId_;

    StreamProfileListUnsafe profileList_;

    std::string             timeoutMsg_;
    std::mutex              timeoutMutex_;
    std::condition_variable timeoutCondition_;
    std::thread             timeoutThread_;
    std::atomic_bool        timeoutThreadRun_;
    std::atomic_bool        hasTimeoutBarrier_;

public:
    STDMETHODIMP QueryInterface(REFIID iid, void **ppv) override;
    STDMETHODIMP_(ULONG) AddRef() override;
    STDMETHODIMP_(ULONG) Release() override;
    STDMETHODIMP OnReadSample(HRESULT /*hrStatus*/, DWORD dwStreamIndex, DWORD /*dwStreamFlags*/, LONGLONG /*llTimestamp*/, IMFSample *sample) override;
    STDMETHODIMP OnEvent(DWORD /*sidx*/, IMFMediaEvent * /*event*/) override;
    STDMETHODIMP OnFlush(DWORD) override;

private:
    long refCount_ = 1;
};

}  // namespace libobsensor
