#pragma once
#include <condition_variable>

#include "RawPhaseConverterDevice.hpp"
#include "platform/usb/backend/Device.hpp"
#include "core/frame/FrameBufferManager.hpp"
#include "core/frame/FrameMemoryPool.hpp"

#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
#include "DepthEngine.h"
#include "k4aplugin.h"
#include "YeatsFrameHdr.h"

#include "dynlib/dynlib.h"
#include "dynlib/global.h"

typedef struct {
    k4a_plugin_t           plugin;
    dynlib_t               handle;
    k4a_register_plugin_fn registerFn;
    volatile bool          loaded;
} deloader_global_context_t;

static void deloader_init_once(deloader_global_context_t *global);
static void deloader_deinit(void);

// Creates a function called deloader_global_context_t_get() which returns the initialized
// singleton global
K4A_DECLARE_GLOBAL(deloader_global_context_t, deloader_init_once);

// static bool verify_plugin(const k4a_plugin_t *plugin) {
//     RETURN_VALUE_IF_ARG(false, plugin == NULL);

//     LOG_INFO("Loaded Depth Engine version: %u.%u.%u", plugin->version.major, plugin->version.minor, plugin->version.patch);

//     // All function pointers must be non NULL
//     RETURN_VALUE_IF_ARG(false, plugin->depth_engine_create_and_initialize == NULL);
//     RETURN_VALUE_IF_ARG(false, plugin->depth_engine_process_frame == NULL);
//     RETURN_VALUE_IF_ARG(false, plugin->depth_engine_get_output_frame_size == NULL);
//     RETURN_VALUE_IF_ARG(false, plugin->depth_engine_destroy == NULL);
//     RETURN_VALUE_IF_ARG(false, plugin->transform_engine_create_and_initialize == NULL);
//     RETURN_VALUE_IF_ARG(false, plugin->transform_engine_process_frame == NULL);
//     RETURN_VALUE_IF_ARG(false, plugin->transform_engine_get_output_frame_size == NULL);
//     RETURN_VALUE_IF_ARG(false, plugin->transform_engine_destroy == NULL);

//     return true;
// }

// Load Depth Engine
static void deloader_init_once(deloader_global_context_t *global) {
    // All members are initialized to zero

    k4a_result_t result = dynlib_create(K4A_PLUGIN_DYNAMIC_LIBRARY_NAME, K4A_PLUGIN_VERSION, &global->handle);
    if(K4A_FAILED(result)) {
        // LOG_ERROR("Failed to Load Depth Engine Plugin (%s). Depth functionality will not work", K4A_PLUGIN_DYNAMIC_LIBRARY_NAME);
        // LOG_ERROR("Make sure the depth engine plugin is in your loaders path", 0);
    }

    if(K4A_SUCCEEDED(result)) {
        result = dynlib_find_symbol(global->handle, K4A_PLUGIN_EXPORTED_FUNCTION, (void **)&global->registerFn);
    }

    if(K4A_SUCCEEDED(result)) {
        // result = K4A_RESULT_FROM_BOOL(global->registerFn(&global->plugin));
        global->registerFn(&global->plugin);
    }

    if(K4A_SUCCEEDED(result)) {
        // result = K4A_RESULT_FROM_BOOL(verify_plugin(&global->plugin));
        // verify_plugin(&global->plugin);
    }

    if(K4A_SUCCEEDED(result)) {
        global->loaded = true;
    }
}

static bool is_plugin_loaded(deloader_global_context_t *global) {
    return global->loaded;
}
#endif

namespace libobsensor {
// Chip defintions
// Since we don't actually have direct access to the chip here, we will need to hardcode a few parameters
const float    CHIP_ADC_UNITS_PER_DEG_C   = 5.45f;  // This is a hardcoded paramter
const uint32_t CHIP_EFUSE_REG_TEMP_CAL_TJ = 20739;  // Read from EFUSE address 0x0B. LSB: Temp Cal Version, MSB: Forced Tj
const uint32_t CHIP_EFUSE_REG_TJ_ADC_VAL  = 1927;   // Read from EFUSE address 0x0C. Temp Sensor ADC value at Tj

struct InputInfo {
    uint16_t systemId;
    uint16_t nRows;
    uint16_t nCols;
    uint16_t nStreams;
    uint16_t nBitsPerSample;
    uint16_t mode;
};

#pragma pack(push, 1)
typedef struct obcTransferHeader {
    uint64_t frameCounter;
    uint16_t extendtionLen;
} ObcTransferHeader;

typedef struct obcMetadataHeader {
    uint64_t timestamp;  // us
    uint16_t width;
    uint16_t height;
    uint32_t size;
} ObcMetadataHeader;

/****aligned 2 bytes****/
typedef struct obcFrameHeader {
    ObcTransferHeader transferHeader;
    ObcMetadataHeader metadataHeader;
} ObcFrameHeader;
#pragma pack(pop)

namespace pal {

class MSDEConverterDevice : public RawPhaseConverterDevice {

public:
    MSDEConverterDevice(std::shared_ptr<const USBSourcePortInfo> portInfo);
    MSDEConverterDevice(std::shared_ptr<UsbDevice> usbDev, std::shared_ptr<const USBSourcePortInfo> portInfo);
    virtual ~MSDEConverterDevice() noexcept;

    virtual void startStream(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) override;

    virtual void stopStream(std::shared_ptr<const VideoStreamProfile> profile) override;

    virtual std::vector<std::shared_ptr<const VideoStreamProfile>> getStreamProfileList() override;

    virtual std::vector<std::shared_ptr<const VideoStreamProfile>> getRawPhaseStreamProfileList() override;

    void setIsPassiveIR(bool isPassiveIR);

    void setTemperatureData(OBDeviceTemperature temperature) {
        std::lock_guard<std::mutex> lk(temperatureMutex_);
        currentTemperature_ = temperature;
    }

    void setNvramDataStreamStopFunc(std::function<void()> stopFunc);

protected:
    virtual void processFrame(VideoFrameObject fo) override;

private:
    // depth engine

    void depthEngineThreadFunc();

    bool initDepthEngine(std::shared_ptr<const VideoStreamProfile> profile);

    void deinitDepthEngine();

#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    k4a_depth_engine_mode_t getDepthEngineMode(std::shared_ptr<const VideoStreamProfile> profile);
#endif

    void processFrameFunc(std::shared_ptr<VideoFrame> videoFrame);

    void startDepthEngineThread();

    void terminateDepthEngineThread();

    void refreshCallback();

    // Nvram
    void initNvramData();

    void stopGetNvramDataStream();

    void onNvramDataCallback(VideoFrameObject fo);

private:
#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    // Depth Engine
    k4a_depth_engine_context_t *depthEngineContext_ = nullptr;

    k4a_depth_engine_mode_t curDepthEngineMode_ = K4A_DEPTH_ENGINE_MODE_UNKNOWN;

#endif

    std::vector<std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>>> profileVector_;

    std::vector<std::shared_ptr<const VideoStreamProfile>> streamProfileList_;

    bool isPassiveIR_ = false;

    // nvram
    uint8_t *nvramData_ = nullptr;

    uint32_t nvramSize_ = 0;

    std::function<void()> stopNvramDataFunc_ = nullptr;

    // temperature
    std::mutex          temperatureMutex_;
    OBDeviceTemperature currentTemperature_{};

    // stream
    std::thread depthEngineThread_;

    std::mutex frameQueueMutex_;

    std::condition_variable frameQueueCV_;

    std::mutex callbackMutex_;

    std::queue<std::shared_ptr<VideoFrame>> videoFrameObjectVec_;

    bool threadExit_ = false;

    std::shared_ptr<IFrameBufferManager> rawphaseFrameBufferManager_;

    std::condition_variable deInitCv_;
};

}  // namespace pal

}  // namespace libobsensor
