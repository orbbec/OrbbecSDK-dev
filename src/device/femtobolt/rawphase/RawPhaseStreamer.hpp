#pragma once

#include <condition_variable>
#include <atomic>
#include <map>
#include <mutex>
#include <queue>
#include "ISourcePort.hpp"
#include "IDeviceComponent.hpp"
#include "depthengine/DepthEngineLoader.hpp"
#include "depthengine/YeatsFrameHdr.h"

// #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
#if 0
#include "dynlib/k4aplugin.h"
#include "dynlib/YeatsFrameHdr.h"

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
// #endif
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

class RawPhaseStreamer : public IDeviceComponent {
public:
    RawPhaseStreamer(IDevice *owner, const std::shared_ptr<IVideoStreamPort> &backend, std::shared_ptr<DepthEngineLoadFactory> &depthEngineLoader);
    virtual ~RawPhaseStreamer() noexcept;

    void start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback);
    void stop(std::shared_ptr<const StreamProfile> sp);

    IDevice *getOwner() const override;

    void setIsPassiveIR(bool isPassiveIR);
    void setNvramDataStreamStopFunc(std::function<void()> stopFunc);

private:
    virtual void parseRawPhaseFrame(std::shared_ptr<Frame> frame);

    // Nvram
    void                  initNvramData();
    void                  onNvramDataCallback(std::shared_ptr<const Frame> frame);
    std::function<void()> stopNvramDataFunc_ = nullptr;
    void                  stopGetNvramDataStream();
    void                  startDepthEngineThread(std::shared_ptr<const StreamProfile> profile);
    void                  terminateDepthEngineThread();
    bool                  initDepthEngine(std::shared_ptr<const StreamProfile> profile);
    void                  depthEngineThreadFunc(std::shared_ptr<const StreamProfile> profile);
    void                  deinitDepthEngine();

    // #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    k4a_depth_engine_mode_t getDepthEngineMode(std::shared_ptr<const StreamProfile> profile);
    // #endif

private:
    IDevice                          *owner_;
    std::shared_ptr<IVideoStreamPort> backend_;

    std::mutex                                                    cbMtx_;
    std::map<std::shared_ptr<const StreamProfile>, FrameCallback> callbacks_;
    std::atomic_bool                                                                     running_;
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>>> profileVector_;
    std::vector<std::shared_ptr<StreamProfile>>                                          streamProfileList_;
    std::shared_ptr<const StreamProfile>                                                 realSp = nullptr;
    // nvram
    uint8_t             *nvramData_   = nullptr;
    uint32_t             nvramSize_   = 0;
    bool                 isPassiveIR_ = false;
    std::recursive_mutex streamMutex_;
    std::mutex           temperatureMutex_;
    OBDeviceTemperature  currentTemperature_{};

    // #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    //  Depth Engine
    k4a_depth_engine_context_t *depthEngineContext_ = nullptr;
    k4a_depth_engine_mode_t     curDepthEngineMode_ = K4A_DEPTH_ENGINE_MODE_UNKNOWN;
    // #endif
    std::shared_ptr<DepthEngineLoadFactory> depthEngineLoader_;
    bool                                    threadExit_ = false;
    std::thread                             depthEngineThread_;
    std::condition_variable                 deInitCv_;
    std::shared_ptr<const StreamProfile>    firmwareDataProfile = nullptr;

    std::queue<std::shared_ptr<Frame>> videoFrameObjectVec_;
    std::mutex                         frameQueueMutex_;
    std::condition_variable            frameQueueCV_;
};

}  // namespace libobsensor