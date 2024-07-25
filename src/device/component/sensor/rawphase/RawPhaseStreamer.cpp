#include "RawPhaseStreamer.hpp"
#include "frame/Frame.hpp"
#include "frame/FrameFactory.hpp"
#include "stream/StreamProfile.hpp"
#include "logger/LoggerInterval.hpp"
#include "stream/StreamProfileFactory.hpp"

namespace libobsensor {
RawPhaseStreamer::RawPhaseStreamer(IDevice *owner, const std::shared_ptr<IVideoStreamPort> &backend, std::shared_ptr<DepthEngineLoadFactory> &depthEngineLoader)
    : owner_(owner), backend_(backend), depthEngineLoader_(depthEngineLoader) {
    profileVector_.push_back({ { 320, 288 }, { 7680, 434 } });
    profileVector_.push_back({ { 640, 576 }, { 7680, 434 } });
    profileVector_.push_back({ { 1024, 1024 }, { 8192, 130 } });
    profileVector_.push_back({ { 512, 512 }, { 8192, 290 } });
    //  updateStreamProfileList();
    initNvramData();

    LOG_DEBUG("RawPhaseStreamer created");
}

void RawPhaseStreamer::updateStreamProfileList() {}

RawPhaseStreamer::~RawPhaseStreamer() noexcept {
    {
        std::lock_guard<std::mutex> lock(cbMtx_);
        callbacks_.clear();
    }

    if(running_) {
        // auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
        // vsPort->stopStream();
        running_ = false;
    }
}

void RawPhaseStreamer::start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) {
#if 1
    {
        std::lock_guard<std::mutex> lock(cbMtx_);
        callbacks_[sp] = callback;
        if(running_) {
            return;
        }
    }

    std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    auto                                   profile    = std::dynamic_pointer_cast<const VideoStreamProfile>(sp);
    auto                                   resolution = std::make_pair(profile->getWidth(), profile->getHeight());
    auto                                   iter =
        std::find_if(profileVector_.begin(), profileVector_.end(),
                     [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.first == resolution; });
    auto valueIter =
        std::find_if(profileVector_.begin(), profileVector_.end(),
                     [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.second == resolution; });
    if(iter != profileVector_.end() && valueIter == profileVector_.end()) {
        // auto compareProfile =
        //   std::make_shared<VideoStreamProfile>(profile->getType(), profile->getFormat(), (*valueIter).first.first, (*valueIter).first.second,
        //   profile->getFps());
        //  profile->setWidth((*valueIter).first.first);
        //  profile->setHeight((*valueIter).first.second);
    }

    // if(profile_ != nullptr
    //    && (profile_->width != profile->width || profile_->height != profile->height || profile_->fps != profile->fps || profile_->format != profile->format))
    //    {
    //     {
    //         std::unique_lock<std::mutex> lock(frameQueueMutex_);
    //         frameCallbacks_.clear();
    //     }
    //     stopStream(profile_);
    //     running_ = false;
    // }

    // #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    auto mode = getDepthEngineMode(profile);
    if(curDepthEngineMode_ != mode) {
        terminateDepthEngineThread();
    }
    // #endif

    //  profile_    = sp;
    std::shared_ptr<const StreamProfile> realSp = nullptr;
    if(iter != profileVector_.end()) {
        auto pair = iter->second;
        realSp    = StreamProfileFactory::createVideoStreamProfile(profile->getType(), profile->getFormat(), pair.first, pair.second, profile->getFps());
    }
    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    vsPort->startStream(realSp, [this](std::shared_ptr<Frame> frame) {
        std::unique_lock<std::mutex> lock(frameQueueMutex_);
        videoFrameObjectVec_.push(frame);
        frameQueueCV_.notify_one();
    });
    startDepthEngineThread(sp);

    running_ = true;
#endif
}

void RawPhaseStreamer::stop(std::shared_ptr<const StreamProfile> sp) {
    {
        std::lock_guard<std::mutex> lock(cbMtx_);
        auto                        iter = callbacks_.find(sp);
        if(iter == callbacks_.end()) {
            throw invalid_value_exception("Stop stream failed, stream profile not found");
        }

        callbacks_.erase(iter);
        if(!callbacks_.empty()) {
            return;
        }
    }
    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    vsPort->stopStream(sp);
    running_ = false;
}

void RawPhaseStreamer::parseRawPhaseFrame(std::shared_ptr<Frame> frame) {
    // #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    if(depthEngineContext_ == nullptr) {
        return;
    }

    InputInfo inputInfo   = { 0 };
    size_t    captureSize = 0, rawFrameSize = 0; /** headerSize = 0*/

    // Parse the metadata from the MIPI header
    YEATS_MIPI_HDR mipiHdr      = { 0 };
    int            mipiHeadSize = sizeof(YEATS_MIPI_HDR) * 2;
    // passive ir
    if(frame->getDataSize() == 4096 * 1154 * 2) {
        mipiHeadSize = sizeof(YEATS_MIPI_HDR);
    }

    auto data = frame->getData();
    // Try reading chip ID to determine if there is padding in header or not
    uint16_t chipId = *(uint16_t *)data;
    if(chipId == 0x5931) {
        // For mode 5, we can read directly into the mipi header
        mipiHdr = *(YEATS_MIPI_HDR *)data;
    }
    else {
        //  int size = sizeof(YEATS_MIPI_HDR) * 2;
        // For mode 4/7, we need to skip every other byte because of padding from Jetson Nano
        std::vector<uint8_t> headerData(sizeof(YEATS_MIPI_HDR) * 2);
        // inFile.read(reinterpret_cast<char *>(headerData.data()), sizeof(YEATS_MIPI_HDR) * 2);
        memcpy(headerData.data(), data, sizeof(YEATS_MIPI_HDR) * 2);
        for(int i = 0; i < sizeof(YEATS_MIPI_HDR); ++i) {
            uint8_t *mipiHdrArray = reinterpret_cast<uint8_t *>(&mipiHdr);
            mipiHdrArray[i]       = headerData[i * 2];
        }
    }

    // Note: if you want to use NFOV_2x2BINNED, capture data using NFOV_UNBINNED and then set the inputInfo.mode = 2
    // Read the relevant info from the mipi header
    inputInfo.mode           = mipiHdr.mode_info.bits.mode;
    inputInfo.nRows          = mipiHdr.frameHeight;
    inputInfo.nCols          = mipiHdr.frameWidth;
    inputInfo.nStreams       = mipiHdr.capture_info.bits.nCaptures + 1;  // zero indexed
    inputInfo.nBitsPerSample = (inputInfo.mode == 5) ? 8 : 16;           // mode 5 uses 8 bit compressed, others use 16 bit float

    captureSize = inputInfo.nRows * inputInfo.nCols * inputInfo.nBitsPerSample / 8;

    rawFrameSize = captureSize * inputInfo.nStreams;

    uint8_t *dstData = (uint8_t *)data + rawFrameSize;
    uint8_t *srcData = (uint8_t *)data + rawFrameSize - (inputInfo.nRows * 2);
    memcpy(dstData, srcData, mipiHeadSize);

    // std::unique_ptr<k4a_depth_engine_context_t, void (*)(k4a_depth_engine_context_t *)> depthEngineContext(
    //     depthEngineContext_, [](k4a_depth_engine_context_t *depthEngineContext) {});

    auto global = depthEngineLoader_->getGlobalContext();

    if(!global->loaded) {
        LOG_ERROR("Failed to load depth engine plugin,init depth engine failed.");
        return;
    }

    size_t outputBytes = global->plugin.depth_engine_get_output_frame_size(depthEngineContext_);

    std::unique_ptr<uint8_t> outputBuffer(new uint8_t[outputBytes]);

    // For every frame, read the RAW data and process it.
    k4a_depth_engine_input_frame_info_t  inputFrameInfo  = { 0 };
    k4a_depth_engine_output_frame_info_t outputFrameInfo = { 0 };

    {

        // Need to add in the laser and sensor temperature info.
        // Read this from the chip EFUSE in combination with header.
        uint32_t Tj               = (CHIP_EFUSE_REG_TEMP_CAL_TJ >> 8);
        float    sensorTempOffset = CHIP_EFUSE_REG_TJ_ADC_VAL - Tj * CHIP_ADC_UNITS_PER_DEG_C;

        {
            std::lock_guard<std::mutex> lk(temperatureMutex_);
            float                       ldmTemp = currentTemperature_.ldmTemp;
            if(ldmTemp == -1.0f) {
                int32_t resvValue      = ((int32_t)mipiHdr.resv[0]) | ((int32_t)mipiHdr.resv[1] << 16);
                int16_t integerPart    = static_cast<uint16_t>(resvValue / 1000);
                int16_t fractionalPart = static_cast<uint16_t>(resvValue % 1000);

                float tmpValue               = static_cast<float>(integerPart) + static_cast<float>(fractionalPart) / 1000.0f;
                inputFrameInfo.laser_temp[0] = tmpValue;
                // LOG_INFO("get temp from ebd:{}", tmpValue);
                // LOG_INFO("origin data:{:x},{:x},{:x},{:x}", mipiHdr.resv[0], mipiHdr.resv[1], mipiHdr.resv[2], mipiHdr.resv[3]);
            }
            else {
                // inputFrameInfo.laser_temp[0] = ldmTemp;  // This should be read directly from the laser

                int32_t resvValue      = ((int32_t)mipiHdr.resv[0]) | ((int32_t)mipiHdr.resv[1] << 16);
                int16_t integerPart    = static_cast<uint16_t>(resvValue / 1000);
                int16_t fractionalPart = static_cast<uint16_t>(resvValue % 1000);

                float tmpValue               = static_cast<float>(integerPart) + static_cast<float>(fractionalPart) / 1000.0f;
                inputFrameInfo.laser_temp[0] = tmpValue;
            }
        }
        inputFrameInfo.laser_temp[1]               = 0;  // This can just be populated with 0
        inputFrameInfo.sensor_temp                 = (((float)mipiHdr.tempSensorADC.bits.adcVal) - sensorTempOffset) / CHIP_ADC_UNITS_PER_DEG_C;
        inputFrameInfo.center_of_exposure_in_ticks = 0;  // This isn't used within the depth engine

        k4a_depth_engine_output_type_t outputType = k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_Z_DEPTH;
        if(isPassiveIR_) {
            outputType = k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_PCM;
        }

        // auto retCode = DepthEngine_ProcessFrame(depthEngineContext.get(), (uint8_t *)fo.frameData + mipiHeadSize, rawFrameSize, outputType,
        // outputBuffer.get(),
        //                                         outputBytes, &outputFrameInfo, &inputFrameInfo);
        // auto global = depthEngineLoader_->getGlobalContext();

        // if(!global->loaded) {
        //     LOG_ERROR("Failed to load depth engine plugin,init depth engine failed.");
        //     return;
        // }
        auto retCode = global->plugin.depth_engine_process_frame(depthEngineContext_, (uint8_t *)data + mipiHeadSize, rawFrameSize, outputType,
                                                                 outputBuffer.get(), outputBytes, &outputFrameInfo, &inputFrameInfo);

        if(K4A_DEPTH_ENGINE_RESULT_SUCCEEDED != retCode) {
            LOG_ERROR("Process frame failed!{}", retCode);
            return;
        }

        // Now we have the output data. They are uint16_t type and AB frame immediately follows Z frame.
        size_t    nPixels     = static_cast<size_t>(outputFrameInfo.output_height) * static_cast<size_t>(outputFrameInfo.output_width);
        uint16_t *outputFrame = reinterpret_cast<uint16_t *>(outputBuffer.get());
        uint16_t *zFrame      = outputFrame;
        uint16_t *abFrame     = outputFrame + nPixels;

        if(outputType == k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_PCM) {
            abFrame = zFrame;
        }

        std::shared_ptr<const libobsensor::VideoStreamProfile> depthStreamProfile;
        std::shared_ptr<const libobsensor::VideoStreamProfile> irStreamProfile;
        for(const auto &iter: callbacks_) {
            if(iter.first->getType() == OB_STREAM_DEPTH) {
                depthStreamProfile = iter.first->as<libobsensor::VideoStreamProfile>();
            }
            else if(iter.first->getType() == OB_STREAM_IR) {
                irStreamProfile = iter.first->as<libobsensor::VideoStreamProfile>();
            }
        }

        auto frameSet = FrameFactory::createFrameSet();
        if(depthStreamProfile && outputType != k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_PCM) {
            auto depthFrame = FrameFactory::createFrameFromStreamProfile(depthStreamProfile);
            depthFrame->updateData((const uint8_t *)zFrame, nPixels * 2);
            depthFrame->setDataSize(nPixels * 2);
            frameSet->pushFrame(depthFrame);
        }

        if(irStreamProfile) {
            auto irFrame = FrameFactory::createFrameFromStreamProfile(irStreamProfile);
            irFrame->updateData((const uint8_t *)zFrame, nPixels * 2);
            irFrame->setDataSize(nPixels * 2);
            frameSet->pushFrame(irFrame);
        }

        outputFrameSet(frameSet);
    }

    // #endif
}

// #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
k4a_depth_engine_mode_t RawPhaseStreamer::getDepthEngineMode(std::shared_ptr<const StreamProfile> profile) {
    auto                    videoProfile = std::dynamic_pointer_cast<const VideoStreamProfile>(profile);
    k4a_depth_engine_mode_t mode         = K4A_DEPTH_ENGINE_MODE_UNKNOWN;
    if((videoProfile->getWidth() == 1024 && videoProfile->getHeight() == 1024) || ((videoProfile->getWidth() == 8192) && videoProfile->getHeight() == 130)
       || (videoProfile->getWidth() == 4096 && videoProfile->getHeight() == 1154)) {
        if(isPassiveIR_) {
            mode = K4A_DEPTH_ENGINE_MODE_PCM;
        }
        else {
            mode = K4A_DEPTH_ENGINE_MODE_MEGA_PIXEL;
        }
    }
    else if((videoProfile->getWidth() == 512 && videoProfile->getHeight() == 512) || (videoProfile->getWidth() == 8192 && videoProfile->getHeight() == 290)) {
        mode = K4A_DEPTH_ENGINE_MODE_QUARTER_MEGA_PIXEL;
    }
    else if((videoProfile->getWidth() == 640 && videoProfile->getHeight() == 576) || (videoProfile->getWidth() == 7680 && videoProfile->getHeight() == 434)) {
        mode = K4A_DEPTH_ENGINE_MODE_LT_NATIVE;
    }
    else if((videoProfile->getWidth() == 320 && videoProfile->getHeight() == 288) || (videoProfile->getWidth() == 7680 && videoProfile->getHeight() == 434)) {
        mode = K4A_DEPTH_ENGINE_MODE_LT_SW_BINNING;
    }

    return mode;
}
// #endif

void RawPhaseStreamer::terminateDepthEngineThread() {
    threadExit_ = true;
    if(depthEngineThread_.joinable()) {
        {
            std::unique_lock<std::mutex> lock(frameQueueMutex_);
            frameQueueCV_.notify_one();
        }
        depthEngineThread_.join();
    }
}

void RawPhaseStreamer::startDepthEngineThread(std::shared_ptr<const StreamProfile> profile) {
    terminateDepthEngineThread();
    threadExit_        = false;
    depthEngineThread_ = std::thread(&RawPhaseStreamer::depthEngineThreadFunc, this, profile);
    std::mutex                   mtx;
    std::unique_lock<std::mutex> lk(mtx);
    // LOG_ERROR("===============deinit cv wait");
    deInitCv_.wait(lk);
    // LOG_ERROR("===============deinit cv wait end");
}

void RawPhaseStreamer::depthEngineThreadFunc(std::shared_ptr<const StreamProfile> profile) {
    uint8_t retry  = 10;
    bool    status = false;
    while(retry-- > 0 && !status) {
        status = initDepthEngine(profile);
        utils::sleepMs(300);
    }
    deInitCv_.notify_all();
    // LOG_ERROR("===============deinit cv notify");

    while(!threadExit_) {
        std::unique_lock<std::mutex> lock(frameQueueMutex_);
        frameQueueCV_.wait(lock, [&]() { return !videoFrameObjectVec_.empty() || threadExit_; });
        if(threadExit_) {
            break;
        }

        auto videoFrame = videoFrameObjectVec_.front();
        videoFrameObjectVec_.pop();
        parseRawPhaseFrame(videoFrame);
    }

    // // deinit
    deinitDepthEngine();
}

bool RawPhaseStreamer::initDepthEngine(std::shared_ptr<const StreamProfile> profile) {
// #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
#if 0
    std::unique_lock<std::recursive_mutex> lk(streamMutex_, std::defer_lock);
    std::ifstream                          in("./Sample_Cal.ccb", std::ifstream::ate | std::ifstream::binary);
    auto                                   ccbBufferSize = static_cast<size_t>(in.tellg());
    in.seekg(0);
    std::unique_ptr<uint8_t> ccbBuffer(new uint8_t[ccbBufferSize]);
    in.read(reinterpret_cast<char *>(ccbBuffer.get()), ccbBufferSize);
    in.close();
#else
    std::unique_lock<std::recursive_mutex> lk(streamMutex_, std::defer_lock);
    if(!nvramData_ || nvramSize_ == 0) {
        LOG_ERROR("Depth engine create and initialize failed! Nvram data is invalid.");
        return false;
    }
    LOG_INFO("Depth engine got nvram data size:{}", nvramSize_);
#endif

    k4a_depth_engine_input_type_t deInputType = k4a_depth_engine_input_type_t::K4A_DEPTH_ENGINE_INPUT_TYPE_UNKNOWN;
    k4a_depth_engine_mode_t       mode        = K4A_DEPTH_ENGINE_MODE_UNKNOWN;

    auto videoProfile = std::dynamic_pointer_cast<const VideoStreamProfile>(profile);
    mode              = getDepthEngineMode(videoProfile);

    deInputType = (mode == K4A_DEPTH_ENGINE_MODE_MEGA_PIXEL) ? k4a_depth_engine_input_type_t::K4A_DEPTH_ENGINE_INPUT_TYPE_8BIT_COMPRESSED_RAW
                                                             : k4a_depth_engine_input_type_t::K4A_DEPTH_ENGINE_INPUT_TYPE_16BIT_RAW;

    if(!depthEngineContext_) {
        curDepthEngineMode_ = mode;

#if 0
        k4a_depth_engine_result_code_t retCode =
            DepthEngine_CreateAndInitialize(&depthEngineContext_, ccbBufferSize, ccbBuffer.get(), mode, deInputType, nullptr, nullptr, nullptr);
#else
        // k4a_depth_engine_result_code_t retCode =
        // DepthEngine_CreateAndInitialize(&depthEngineContext_, nvramSize_, nvramData_, mode, deInputType, nullptr, nullptr, nullptr);

        auto global = depthEngineLoader_->getGlobalContext();

        if(!global->loaded) {
            LOG_ERROR("Failed to load depth engine plugin,init depth engine failed.");
            return false;
        }
        LOG_INFO("use dynlib load depthengine lib......");
        k4a_depth_engine_result_code_t retCode =
            global->plugin.depth_engine_create_and_initialize(&depthEngineContext_, nvramSize_, nvramData_, mode, deInputType, nullptr, nullptr, nullptr);
        // k4a_depth_engine_result_code_t retCode = K4A_DEPTH_ENGINE_RESULT_SUCCEEDED;
#endif

        if(K4A_DEPTH_ENGINE_RESULT_SUCCEEDED != retCode) {
            LOG_ERROR("Depth engine create and initialize failed,retCode:{}", retCode);
            return false;
        }
        else {
            LOG_INFO("Depth engine init succeed!");
        }
    }
    else {
        LOG_INFO("Depth engine already inited!");
    }
    // #endif
    return true;
}

void RawPhaseStreamer::deinitDepthEngine() {
    // #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    if(depthEngineContext_ == nullptr)
        return;

    // DepthEngine_Destroy(&depthEngineContext_);
    auto global = depthEngineLoader_->getGlobalContext();
    if(!global->loaded) {
        LOG_ERROR("Failed to load depth engine plugin,init depth engine failed.");
        return;
    }
    global->plugin.depth_engine_destroy(&depthEngineContext_);
    depthEngineContext_ = nullptr;
    // #endif
}

void RawPhaseStreamer::outputFrameSet(std::shared_ptr<Frame> frame) {
    if(!frame) {
        return;
    }

    std::lock_guard<std::mutex> lock(cbMtx_);
    for(auto &callback: callbacks_) {
        std::shared_ptr<const Frame> callbackFrame = frame;
        if(frame->is<FrameSet>()) {
            auto frameSet = frame->as<FrameSet>();
            callbackFrame = frameSet->getFrame(utils::mapStreamTypeToFrameType(callback.first->getType()));
            if(!callbackFrame) {
                continue;
            }
        }

        if(callbackFrame->getFormat() != callback.first->getFormat()) {
            continue;
        }
        callback.second(callbackFrame);
    }
}

IDevice *RawPhaseStreamer::getOwner() const {
    return owner_;
}

void RawPhaseStreamer::initNvramData() {
    // #if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)

    auto global = depthEngineLoader_->getGlobalContext();

    if(!global->loaded) {
        LOG_ERROR("Failed to load depth engine plugin");
    }
    else {
        LOG_INFO("Succeed to load depth engine plugin");
    }

    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    if(!vsPort) {
        throw invalid_value_exception("Backend is not a valid IVideoStreamPort");
    }
    auto profileList = vsPort->getStreamProfileList();

    for(auto &sp: profileList) {
        auto profile = std::dynamic_pointer_cast<const VideoStreamProfile>(sp);
        if(profile->getWidth() == 1024 && profile->getHeight() == 512 && profile->getFps() == 5) {
            // firmwareDataProfile=StreamProfileFactory::createVideoStreamProfile(OB_STREAM_RAW_PHASE,profile->getFormat(), profile->getWidth(),
            //                                                             profile->getHeight(), profile->getFps());
            firmwareDataProfile = sp;
            break;
        }
    }
    if(firmwareDataProfile) {
        vsPort->startStream(firmwareDataProfile, [&](std::shared_ptr<const Frame> frame) { onNvramDataCallback(frame); });
    }
    else {
        LOG_ERROR("Can not find firmware data profile.");
    }

    // #endif
}

void RawPhaseStreamer::onNvramDataCallback(std::shared_ptr<const Frame> frame) {
    std::unique_lock<std::recursive_mutex> lk(streamMutex_, std::defer_lock);
    if(lk.try_lock()) {
        ObcFrameHeader *frameHeader = (ObcFrameHeader *)frame->getData();
        nvramSize_                  = frameHeader->metadataHeader.size;
        if(!nvramData_) {
            nvramData_ = new uint8_t[nvramSize_];
        }
        memcpy(nvramData_, (uint8_t *)frame->getData() + sizeof(ObcFrameHeader), nvramSize_);

        if(nvramData_ != nullptr && nvramSize_ != 0 && firmwareDataProfile != nullptr) {
            auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
            vsPort->stopStream(firmwareDataProfile);
            utils::sleepMs(300);
            initialized_ = true;
        }
    }
}

void RawPhaseStreamer::setIsPassiveIR(bool isPassiveIR) {
    isPassiveIR_ = isPassiveIR;

    for(auto it = profileVector_.begin(); it != profileVector_.end();) {
        if(it->first.first == 1024 && it->first.second == 1024) {
            it = profileVector_.erase(it);
        }
        else {
            ++it;
        }
    }

    if(isPassiveIR) {
        profileVector_.push_back({ { 1024, 1024 }, { 8192, 130 } });
    }
    else {
        profileVector_.push_back({ { 1024, 1024 }, { 4096, 1154 } });
    }
    //  streamProfileList_.clear();

    // streamProfileList_ = getStreamProfileList();
}

}  // namespace libobsensor