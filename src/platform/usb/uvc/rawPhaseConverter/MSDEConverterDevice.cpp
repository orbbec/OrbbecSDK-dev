#include "MSDEConverterDevice.hpp"

#ifdef WIN32
#include "platform/usb/uvc/WmfUvcDevicePort.hpp"
#else
#include "platform/usb/uvc/ObLibuvcDevicePort.hpp"
#endif  //

#include <common/exception/ObException.hpp>
#include "common/logger/Logger.hpp"

#include <iostream>
#include <sstream>
#include <fstream>

namespace libobsensor {
namespace pal {

MSDEConverterDevice::MSDEConverterDevice(std::shared_ptr<const USBSourcePortInfo> portInfo) : RawPhaseConverterDevice(portInfo) {
#ifdef WIN32
    srcUvcPort_ = std::make_shared<WmfUvcDevicePort>(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
    profileVector_.push_back({ { 320, 288 }, { 7680, 434 } });
    profileVector_.push_back({ { 640, 576 }, { 7680, 434 } });
    profileVector_.push_back({ { 1024, 1024 }, { 8192, 130 } });
    profileVector_.push_back({ { 512, 512 }, { 8192, 290 } });

#endif
    initNvramData();
}
MSDEConverterDevice::MSDEConverterDevice(std::shared_ptr<UsbDevice> usbDev, std::shared_ptr<const USBSourcePortInfo> portInfo)
    : RawPhaseConverterDevice(portInfo) {
#ifndef WIN32
    profileVector_.push_back({ { 320, 288 }, { 7680, 434 } });
    profileVector_.push_back({ { 640, 576 }, { 7680, 434 } });
    profileVector_.push_back({ { 1024, 1024 }, { 8192, 130 } });
    profileVector_.push_back({ { 512, 512 }, { 8192, 290 } });
    srcUvcPort_ = std::make_shared<ObLibuvcDevicePort>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
#endif  // ! WIN32
    initNvramData();
}

MSDEConverterDevice::~MSDEConverterDevice() noexcept {
    terminateDepthEngineThread();

    if(nvramData_) {
        delete[] nvramData_;
        nvramData_ = nullptr;
    }
}

bool MSDEConverterDevice::initDepthEngine(std::shared_ptr<const VideoStreamProfile> profile) {
#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
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

    mode = getDepthEngineMode(profile);

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

        deloader_global_context_t *global = deloader_global_context_t_get();

        if(!is_plugin_loaded(global)) {
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
#endif
    return true;
}

void MSDEConverterDevice::deinitDepthEngine() {
#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    if(depthEngineContext_ == nullptr)
        return;

    // DepthEngine_Destroy(&depthEngineContext_);
    deloader_global_context_t *global = deloader_global_context_t_get();

    if(!is_plugin_loaded(global)) {
        LOG_ERROR("Failed to load depth engine plugin,init depth engine failed.");
        return;
    }
    global->plugin.depth_engine_destroy(&depthEngineContext_);
    depthEngineContext_ = nullptr;
#endif
}

#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
k4a_depth_engine_mode_t MSDEConverterDevice::getDepthEngineMode(std::shared_ptr<const VideoStreamProfile> profile) {
    k4a_depth_engine_mode_t mode = K4A_DEPTH_ENGINE_MODE_UNKNOWN;
    if((profile->width == 1024 && profile->height == 1024) || ((profile->width == 8192) && profile->height == 130)
       || (profile->width == 4096 && profile->height == 1154)) {
        if(isPassiveIR_) {
            mode = K4A_DEPTH_ENGINE_MODE_PCM;
        }
        else {
            mode = K4A_DEPTH_ENGINE_MODE_MEGA_PIXEL;
        }
    }
    else if((profile->width == 512 && profile->height == 512) || (profile->width == 8192 && profile->height == 290)) {
        mode = K4A_DEPTH_ENGINE_MODE_QUARTER_MEGA_PIXEL;
    }
    else if((profile->width == 640 && profile->height == 576) || (profile->width == 7680 && profile->height == 434)) {
        mode = K4A_DEPTH_ENGINE_MODE_LT_NATIVE;
    }
    else if((profile->width == 320 && profile->height == 288) || (profile->width == 7680 && profile->height == 434)) {
        mode = K4A_DEPTH_ENGINE_MODE_LT_SW_BINNING;
    }

    return mode;
}
#endif

void deleteUnsignedIntPtr(unsigned int *data) {
    if(data != nullptr) {
        delete data;
        data = nullptr;
    }
}

void MSDEConverterDevice::processFrame(VideoFrameObject fo) {
    std::unique_lock<std::mutex> lock(frameQueueMutex_);
    if(videoFrameObjectVec_.size() <= 10) {
        auto frame = rawphaseFrameBufferManager_->acquireFrame();
        if(frame) {
            auto videoFrame = frame->as<VideoFrame>();
            videoFrame->setNumber(fo.index);
            videoFrame->updateData((const uint8_t *)fo.frameData, fo.frameSize);
            videoFrame->updateMetadata((const uint8_t *)fo.metadata, fo.metadataSize);
            videoFrame->updateScrData((const uint8_t *)fo.scrDataBuf, fo.scrDataSize);
            videoFrame->setTimeStampUs(fo.deviceTime * 1000);
            videoFrame->setSystemTimeStampUs(fo.systemTime);
            videoFrame->setFormat(fo.format);
            videoFrameObjectVec_.push(videoFrame);
        }
    }
    frameQueueCV_.notify_one();
}

void MSDEConverterDevice::processFrameFunc(std::shared_ptr<VideoFrame> videoFrame) {
    VideoFrameObject fo;
    fo.index        = videoFrame->getNumber();
    fo.systemTime   = videoFrame->getSystemTimeStampUs();
    fo.deviceTime   = videoFrame->getTimeStamp();
    fo.format       = videoFrame->getFormat();
    fo.frameSize    = videoFrame->getDataSize();
    fo.frameData    = videoFrame->getData();
    fo.metadataSize = videoFrame->getMetadataSize();
    fo.metadata     = videoFrame->getMetadata();
    fo.scrDataBuf   = videoFrame->getScrData();
    fo.scrDataSize  = videoFrame->getScrDataSize();

    if(frameCallbacks_[OB_STREAM_RAW_PHASE]) {
        frameCallbacks_[OB_STREAM_RAW_PHASE](fo);
    }

#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    if(!depthEngineContext_) {
        return;
    }

    InputInfo inputInfo   = { 0 };
    size_t    captureSize = 0, headerSize = 0, rawFrameSize = 0;

#if 1

    // FILE *fp = fopen("./640RawPhase.raw", "wb");
    // fwrite(fo.frameData, 1, fo.frameSize, fp);
    // fflush(fp);
    // fclose(fp);

    // Parse the metadata from the MIPI header
    YEATS_MIPI_HDR mipiHdr;
    int            mipiHeadSize = sizeof(YEATS_MIPI_HDR) * 2;
    // passive ir
    if(fo.frameSize == 4096 * 1154 * 2) {
        mipiHeadSize = sizeof(YEATS_MIPI_HDR);
    }

    // Try reading chip ID to determine if there is padding in header or not
    uint16_t chipId = *(uint16_t *)fo.frameData;

    if(chipId == 0x5931) {
        // For mode 5, we can read directly into the mipi header
        mipiHdr = *(YEATS_MIPI_HDR *)fo.frameData;
    }
    else {
        int size = sizeof(YEATS_MIPI_HDR) * 2;
        // For mode 4/7, we need to skip every other byte because of padding from Jetson Nano
        std::vector<uint8_t> headerData(sizeof(YEATS_MIPI_HDR) * 2);
        // inFile.read(reinterpret_cast<char *>(headerData.data()), sizeof(YEATS_MIPI_HDR) * 2);
        memcpy(headerData.data(), fo.frameData, sizeof(YEATS_MIPI_HDR) * 2);
        for(int i = 0; i < sizeof(YEATS_MIPI_HDR); ++i) {
            uint8_t *mipiHdrArray = reinterpret_cast<uint8_t *>(&mipiHdr);
            mipiHdrArray[i]       = headerData[i * 2];
        }
    }

    // Note: if you want to use NFOV_2x2BINNED, capture data using NFOV_UNBINNED and then set the inputInfo.mode = 2
    // Read the relevant info from the mipi header
    inputInfo.mode           = mipiHdr.mode_info.mode;
    inputInfo.nRows          = mipiHdr.frameHeight;
    inputInfo.nCols          = mipiHdr.frameWidth;
    inputInfo.nStreams       = mipiHdr.capture_info.nCaptures + 1;  // zero indexed
    inputInfo.nBitsPerSample = (inputInfo.mode == 5) ? 8 : 16;      // mode 5 uses 8 bit compressed, others use 16 bit float

    captureSize = inputInfo.nRows * inputInfo.nCols * inputInfo.nBitsPerSample / 8;

    rawFrameSize = captureSize * inputInfo.nStreams;

    // 由于SSD268G SOC原因，会导致数据部分最后一行的最后320个字节数据异常，导致深度引擎计算异常，因此需要增加此规避方案
    uint8_t *dstData = (uint8_t *)fo.frameData + rawFrameSize;
    uint8_t *srcData = (uint8_t *)fo.frameData + rawFrameSize - (inputInfo.nRows * 2);
    memcpy(dstData, srcData, mipiHeadSize);

    std::unique_ptr<k4a_depth_engine_context_t, void (*)(k4a_depth_engine_context_t *)> depthEngineContext(
        depthEngineContext_, [](k4a_depth_engine_context_t *depthEngineContext) {});

    // The output buffer will contain the Z and AB images.
    // size_t                   outputBytes = DepthEngine_GetOutputFrameSize(depthEngineContext.get());

    deloader_global_context_t *global = deloader_global_context_t_get();

    if(!is_plugin_loaded(global)) {
        LOG_ERROR("Failed to load depth engine plugin,init depth engine failed.");
        return;
    }

    size_t outputBytes = global->plugin.depth_engine_get_output_frame_size(depthEngineContext.get());

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
                int16_t integerPart    = resvValue / 1000;
                int16_t fractionalPart = resvValue % 1000;

                float tmpValue               = static_cast<float>(integerPart) + static_cast<float>(fractionalPart) / 1000.0f;
                inputFrameInfo.laser_temp[0] = tmpValue;
                // LOG_INFO("get temp from ebd:{}", tmpValue);
                // LOG_INFO("origin data:{:x},{:x},{:x},{:x}", mipiHdr.resv[0], mipiHdr.resv[1], mipiHdr.resv[2], mipiHdr.resv[3]);
            }
            else {
                inputFrameInfo.laser_temp[0] = ldmTemp;  // This should be read directly from the laser
            }
        }
        inputFrameInfo.laser_temp[1]               = 0;  // This can just be populated with 0
        inputFrameInfo.sensor_temp                 = (((float)mipiHdr.tempSensorADC.adcVal) - sensorTempOffset) / CHIP_ADC_UNITS_PER_DEG_C;
        inputFrameInfo.center_of_exposure_in_ticks = 0;  // This isn't used within the depth engine

        k4a_depth_engine_output_type_t outputType = k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_Z_DEPTH;
        if(isPassiveIR_) {
            outputType = k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_PCM;
        }

        // auto retCode = DepthEngine_ProcessFrame(depthEngineContext.get(), (uint8_t *)fo.frameData + mipiHeadSize, rawFrameSize, outputType,
        // outputBuffer.get(),
        //                                         outputBytes, &outputFrameInfo, &inputFrameInfo);
        deloader_global_context_t *global = deloader_global_context_t_get();

        if(!is_plugin_loaded(global)) {
            LOG_ERROR("Failed to load depth engine plugin,init depth engine failed.");
            return;
        }
        auto retCode = global->plugin.depth_engine_process_frame(depthEngineContext.get(), (uint8_t *)fo.frameData + mipiHeadSize, rawFrameSize, outputType,
                                                                 outputBuffer.get(), outputBytes, &outputFrameInfo, &inputFrameInfo);

        if(K4A_DEPTH_ENGINE_RESULT_SUCCEEDED != retCode) {
            LOG_ERROR("Process frame failed!");
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

        if(frameCallbacks_[OB_STREAM_DEPTH] && outputType != k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_PCM) {
            pal::VideoFrameObject depthFrame;
            depthFrame.copyInfo(fo);
            depthFrame.frameSize = (uint32_t)(nPixels * 2);
            memcpy((uint8_t *)depthFrame.frameData, (uint8_t *)zFrame, depthFrame.frameSize);
            frameCallbacks_[OB_STREAM_DEPTH](depthFrame);
        }

        if(frameCallbacks_[OB_STREAM_IR]) {
            pal::VideoFrameObject irFrame;
            irFrame.copyInfo(fo);
            irFrame.frameSize = (uint32_t)(nPixels * 2);
            memcpy((uint8_t *)irFrame.frameData, (uint8_t *)abFrame, irFrame.frameSize);
            frameCallbacks_[OB_STREAM_IR](irFrame);
        }
    }
#else
    std::ifstream inFile;
    inFile.open("./512RawPhase.raw", std::ios::in | std::ios::binary);
    const auto begin = inFile.tellg();

    // Parse the metadata from the MIPI header
    YEATS_MIPI_HDR mipiHdr;

    // Try reading chip ID to determine if there is padding in header or not
    uint16_t chipId;
    inFile.read(reinterpret_cast<char *>(&chipId), 2);
    inFile.seekg(0);

    if(chipId == 0x5931) {
        // For mode 5, we can read directly into the mipi header
        inFile.read(reinterpret_cast<char *>(&mipiHdr), sizeof(YEATS_MIPI_HDR));
    }
    else {
        int size = sizeof(YEATS_MIPI_HDR) * 2;
        // For mode 4/7, we need to skip every other byte because of padding from Jetson Nano
        std::vector<uint8_t> headerData(sizeof(YEATS_MIPI_HDR) * 2);
        inFile.read(reinterpret_cast<char *>(headerData.data()), sizeof(YEATS_MIPI_HDR) * 2);
        for(int i = 0; i < sizeof(YEATS_MIPI_HDR); ++i) {
            uint8_t *mipiHdrArray = reinterpret_cast<uint8_t *>(&mipiHdr);
            mipiHdrArray[i]       = headerData[i * 2];
        }
    }

    // Note: if you want to use NFOV_2x2BINNED, capture data using NFOV_UNBINNED and then set the inputInfo.mode = 2
    // Read the relevant info from the mipi header
    inputInfo.mode           = mipiHdr.mode_info.mode;
    inputInfo.nRows          = mipiHdr.frameHeight;
    inputInfo.nCols          = mipiHdr.frameWidth;
    inputInfo.nStreams       = mipiHdr.capture_info.nCaptures + 1;  // zero indexed
    inputInfo.nBitsPerSample = (inputInfo.mode == 5) ? 8 : 16;      // mode 5 uses 8 bit compressed, others use 16 bit float

    // Read size of bin file to determine how many frames are saved
    inFile.seekg(0, std::ios::end);
    const auto end      = inFile.tellg();
    size_t     fileSize = end - begin;

    captureSize = inputInfo.nRows * inputInfo.nCols * inputInfo.nBitsPerSample / 8;

    // Header size is based off of the supported modes
    headerSize = (inputInfo.mode == 2)   ? 0x3C00
                 : (inputInfo.mode == 3) ? 0x4000
                 : (inputInfo.mode == 4) ? 0x3C00
                 : (inputInfo.mode == 5) ? 0x2000
                 : (inputInfo.mode == 7) ? 0x4000
                                         : 0;

    // Only a single header per frame
    inputFrameSize = captureSize * inputInfo.nStreams + headerSize;
    std::unique_ptr<k4a_depth_engine_context_t, void (*)(k4a_depth_engine_context_t *)> depthEngineContext(
        depthEngineContext_, [](k4a_depth_engine_context_t *depthEngineContext) {});
    // std::unique_ptr<k4a_depth_engine_context_t> depthEngineContext(depthEngineContext_);

    // Create buffers to hold input and output data.
    size_t nInputBytes = static_cast<size_t>(inputInfo.nRows) * static_cast<size_t>(inputInfo.nCols) * static_cast<size_t>(inputInfo.nStreams)
                         * static_cast<size_t>(inputInfo.nBitsPerSample) / 8;
    std::unique_ptr<uint8_t> inputBuffer(new uint8_t[nInputBytes]);

    // The output buffer will contain the Z and AB images.
    size_t                   nOutputBytes = DepthEngine_GetOutputFrameSize(depthEngineContext.get());
    std::unique_ptr<uint8_t> outputBuffer(new uint8_t[nOutputBytes]);

    // For every frame, read the RAW data and process it.
    k4a_depth_engine_input_frame_info_t  inputFrameInfo  = { 0 };
    k4a_depth_engine_output_frame_info_t outputFrameInfo = { 0 };

    {
        // Read raw frames. Set mode to invalid value and parse it out of the metadata.
        inputFrameInfo  = { 0 };
        outputFrameInfo = { 0 };

        // Need to add in the laser and sensor temperature info.
        // Read this from the chip EFUSE in combination with header.
        uint32_t Tj               = (CHIP_EFUSE_REG_TEMP_CAL_TJ >> 8);
        float    sensorTempOffset = CHIP_EFUSE_REG_TJ_ADC_VAL - Tj * CHIP_ADC_UNITS_PER_DEG_C;

        // Seek to current frame and read data
        inFile.seekg(0);

        // Read MIPI Header to get temperature reading
        if(inputInfo.mode == 5) {
            // For mode 5, we can read directly into the mipi header
            inFile.read(reinterpret_cast<char *>(&mipiHdr), sizeof(YEATS_MIPI_HDR));
        }
        else {
            // For mode 4/7, we need to skip every other byte because of padding from Jetson Nano
            std::vector<uint8_t> headerData(sizeof(YEATS_MIPI_HDR) * 2);
            inFile.read(reinterpret_cast<char *>(headerData.data()), sizeof(YEATS_MIPI_HDR) * 2);
            for(int i = 0; i < sizeof(YEATS_MIPI_HDR); ++i) {
                uint8_t *mipiHdrArray = reinterpret_cast<uint8_t *>(&mipiHdr);
                mipiHdrArray[i]       = headerData[i * 2];
            }
        }

        // Read data for current frame
        // inFile.seekg(inputFrameSize* frameID + headerSize);
        inFile.seekg(320);
        inFile.read(reinterpret_cast<char *>(inputBuffer.get()), captureSize * 9);

        inputFrameInfo.laser_temp[0]               = 25;  // This should be read directly from the laser
        inputFrameInfo.laser_temp[1]               = 0;   // This can just be populated with 0
        inputFrameInfo.sensor_temp                 = (((float)mipiHdr.tempSensorADC.adcVal) - sensorTempOffset) / CHIP_ADC_UNITS_PER_DEG_C;
        inputFrameInfo.center_of_exposure_in_ticks = 0;  // This isn't used within the depth engine

        auto t0 = std::chrono::steady_clock::now();

        // Process the input data
        auto retCode = DepthEngine_ProcessFrame(depthEngineContext.get(), inputBuffer.get(), nInputBytes,
                                                k4a_depth_engine_output_type_t::K4A_DEPTH_ENGINE_OUTPUT_TYPE_Z_DEPTH, outputBuffer.get(), nOutputBytes,
                                                &outputFrameInfo, &inputFrameInfo);

        auto t1 = std::chrono::steady_clock::now();
        if(K4A_DEPTH_ENGINE_RESULT_SUCCEEDED != retCode) {
            // LOG_ERROR("Process frame failed!");
            return;
        }

        // Now we have the output data. They are uint16_t type and AB frame immediately follows Z frame.
        size_t    nPixels = static_cast<size_t>(outputFrameInfo.output_height) * static_cast<size_t>(outputFrameInfo.output_width);
        uint16_t *zFrame  = reinterpret_cast<uint16_t *>(outputBuffer.get());
        uint16_t *abFrame = zFrame + nPixels;

        if(depthCallBack_) {
            pal::VideoFrameObject depthFrame;
            depthFrame.copyInfo(fo);
            depthFrame.frameSize = (uint32_t)(nPixels * 2);
            memcpy((uint8_t *)depthFrame.frameData, (uint8_t *)zFrame, depthFrame.frameSize);
            depthCallBack_(depthFrame);
        }

        if(irCallBack_) {
            pal::VideoFrameObject irFrame;
            irFrame.copyInfo(fo);
            irFrame.frameSize = (uint32_t)(nPixels * 2);
            memcpy((uint8_t *)irFrame.frameData, (uint8_t *)abFrame, irFrame.frameSize);
            irCallBack_(irFrame);
        }
    }
#endif
#endif
}

void MSDEConverterDevice::startStream(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) {
    std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    auto                                   resolution = std::make_pair(profile->width, profile->height);
    auto                                   iter =
        std::find_if(profileVector_.begin(), profileVector_.end(),
                     [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.first == resolution; });
    auto valueIter =
        std::find_if(profileVector_.begin(), profileVector_.end(),
                     [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.second == resolution; });
    if(iter == profileVector_.end() && valueIter != profileVector_.end()) {
        auto compareProfile =
            std::make_shared<VideoStreamProfile>(profile->streamType, profile->format, (*valueIter).first.first, (*valueIter).first.second, profile->fps);
        profile = compareProfile;
    }

    if(profile_ != nullptr
       && (profile_->width != profile->width || profile_->height != profile->height || profile_->fps != profile->fps || profile_->format != profile->format)) {
        {
            std::unique_lock<std::mutex> lock(frameQueueMutex_);
            frameCallbacks_.clear();
        }
        stopStream(profile_);
        streamStart_ = false;
    }

#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    auto mode = getDepthEngineMode(profile);
    if(curDepthEngineMode_ != mode) {
        terminateDepthEngineThread();
    }
#endif

    profile_ = profile;

    if(srcUvcPort_) {
        if(streamStart_) {
            std::unique_lock<std::mutex> lock(frameQueueMutex_);
            frameCallbacks_[profile->streamType] = callback;
            return;
        }
        std::shared_ptr<VideoStreamProfile> realProfile = nullptr;
        if(iter != profileVector_.end()) {
            auto pair           = iter->second;
            realProfile         = std::make_shared<VideoStreamProfile>(profile->streamType, profile->format, profile->width, profile->height, profile->fps);
            realProfile->width  = pair.first;
            realProfile->height = pair.second;
            LOG_INFO("Start real profile,width:{0} height:{1}", realProfile->width, realProfile->height);
            srcUvcPort_->startStream(realProfile, [&](pal::VideoFrameObject fo) { onFrameRawDataCallback(fo); });
            {
                std::unique_lock<std::mutex> lock(frameQueueMutex_);
                frameCallbacks_[realProfile->streamType] = callback;
            }
            streamStart_ = true;
        }
        else {
            if(valueIter != profileVector_.end()) {
                realProfile = std::make_shared<VideoStreamProfile>(profile->streamType, profile->format, profile->width, profile->height, profile->fps);
                realProfile->width  = resolution.first;
                realProfile->height = resolution.second;
                LOG_INFO("Start real profile,width:{0} height:{1}", realProfile->width, realProfile->height);
                srcUvcPort_->startStream(realProfile, [&](pal::VideoFrameObject fo) { onFrameRawDataCallback(fo); });
                {
                    std::unique_lock<std::mutex> lock(frameQueueMutex_);
                    frameCallbacks_[realProfile->streamType] = callback;
                }
                streamStart_ = true;
            }
            else {
                LOG_ERROR("Can not find the real profile.");
            }
            // srcUvcPort_->startStream(profile, [&](pal::VideoFrameObject fo) { onFrameRawDataCallback(fo); });
            // frameCallbacks_[profile->streamType] = callback;
            // streamStart_                         = true;
        }

        startDepthEngineThread();
        if(rawphaseFrameBufferManager_ == nullptr) {
            auto memoryPool             = libobsensor::FrameMemoryPool::getInstance();
            rawphaseFrameBufferManager_ = memoryPool->createFrameBufferManager(OB_FRAME_RAW_PHASE, realProfile->width * realProfile->height * 2);
        }
    }
}

void MSDEConverterDevice::stopStream(std::shared_ptr<const VideoStreamProfile> profile) {
    std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    if(srcUvcPort_ && streamStart_) {
        {
            std::unique_lock<std::mutex>                         lock(frameQueueMutex_);
            std::map<OBStreamType, VideoFrameCallback>::iterator iter = frameCallbacks_.begin();
            while(iter != frameCallbacks_.end()) {
                if((*iter).first == profile->streamType) {
                    iter = frameCallbacks_.erase(iter);
                    continue;
                }
                iter++;
            }

            iter = frameCallbacks_.begin();
            while(iter != frameCallbacks_.end()) {
                if((*iter).second == nullptr) {
                    iter = frameCallbacks_.erase(iter);
                    continue;
                }
                iter++;
            }
        }
        if(frameCallbacks_.empty()) {
            auto resolution = std::make_pair(profile->width, profile->height);
            auto iter       = std::find_if(
                profileVector_.begin(), profileVector_.end(),
                [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.first == resolution; });

            if(iter != profileVector_.end()) {
                auto pair        = iter->second;
                auto realProfile = std::make_shared<VideoStreamProfile>(profile->streamType, profile->format, profile->width, profile->height, profile->fps);
                realProfile->width  = pair.first;
                realProfile->height = pair.second;
                LOG_INFO("Stop real profile,width:{0}, height:{1}", realProfile->width, realProfile->height);
                srcUvcPort_->stopStream(realProfile);
                streamStart_ = false;
            }
            else {
                auto valueIter = std::find_if(
                    profileVector_.begin(), profileVector_.end(),
                    [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.second == resolution; });
                if(valueIter != profileVector_.end()) {
                    LOG_INFO("Stop real profile,width:{0} height:{1}", profile->width, profile->height);
                    srcUvcPort_->stopStream(profile);
                    streamStart_ = false;
                }
                else {
                    LOG_WARN("Can not find the real profile.");
                }
            }

            if(rawphaseFrameBufferManager_) {
                rawphaseFrameBufferManager_.reset();
                rawphaseFrameBufferManager_ = nullptr;
            }
        }
    }
}

std::vector<std::shared_ptr<const VideoStreamProfile>> MSDEConverterDevice::getStreamProfileList() {
    auto profileList = srcUvcPort_->getStreamProfileList();
    for(auto profile: profileList) {
        auto resolution = std::make_pair(profile->width, profile->height);

        auto iter = std::find_if(
            profileVector_.begin(), profileVector_.end(),
            [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.second == resolution; });
        if(iter != profileVector_.end()) {
            auto newProfile    = std::make_shared<VideoStreamProfile>(profile->streamType, profile->format, profile->width, profile->height, profile->fps);
            auto pair          = (*iter).first;
            newProfile->width  = pair.first;
            newProfile->height = pair.second;

            if(std::find_if(streamProfileList_.begin(), streamProfileList_.end(),
                            [newProfile](const std::shared_ptr<VideoStreamProfile> &cmpProfile) { return *cmpProfile == *newProfile; })
               != streamProfileList_.end()) {
                continue;
            }

            if(newProfile->width == 320 && newProfile->height == 288) {
                auto spProfile   = std::make_shared<VideoStreamProfile>(profile->streamType, profile->format, profile->width, profile->height, profile->fps);
                spProfile->width = 640;
                spProfile->height = 576;
                streamProfileList_.push_back(spProfile);
            }

            streamProfileList_.push_back(newProfile);
        }
    }

    return streamProfileList_;
}

std::vector<std::shared_ptr<const VideoStreamProfile>> MSDEConverterDevice::getRawPhaseStreamProfileList() {
    auto                      profileList = srcUvcPort_->getStreamProfileList();
    std::vector<std::shared_ptr<const VideoStreamProfile>> retProfileList;
    for(auto profile: profileList) {
        if(profile->width == 1024 && profile->height == 512) {
            continue;
        }
        else {
            retProfileList.push_back(profile);
        }
    }
    return retProfileList;
}

void MSDEConverterDevice::setIsPassiveIR(bool isPassiveIR) {
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
    streamProfileList_.clear();

    streamProfileList_ = getStreamProfileList();
}

void MSDEConverterDevice::initNvramData() {
#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)

    deloader_global_context_t *global = deloader_global_context_t_get();

    if(!is_plugin_loaded(global)) {
        LOG_ERROR("Failed to load depth engine plugin");
    }
    else {
        LOG_INFO("Succeed to load depth engine plugin");
    }

    if(srcUvcPort_) {
        auto                                   profileList         = srcUvcPort_->getStreamProfileList();
        std::shared_ptr<VideoStreamProfile> firmwareDataProfile = nullptr;
        for(auto profile: profileList) {
            if(profile->width == 1024 && profile->height == 512 && profile->fps == 5) {
                firmwareDataProfile = profile;
                break;
            }
        }
        if(firmwareDataProfile != nullptr) {
            srcUvcPort_->startStream(firmwareDataProfile, [&](pal::VideoFrameObject fo) { onNvramDataCallback(fo); });
        }
        else {
            LOG_ERROR("Can not find firmware data profile.");
        }
    }
#endif
}

void MSDEConverterDevice::stopGetNvramDataStream() {
    std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    if(srcUvcPort_) {
        auto                                   profileList         = srcUvcPort_->getStreamProfileList();
        std::shared_ptr<VideoStreamProfile> firmwareDataProfile = nullptr;
        for(auto profile: profileList) {
            if(profile->width == 1024 && profile->height == 512 && profile->fps == 5) {
                firmwareDataProfile = profile;
                break;
            }
        }
        if(firmwareDataProfile != nullptr) {
            srcUvcPort_->stopStream(firmwareDataProfile);
            if(stopNvramDataFunc_) {
                stopNvramDataFunc_();
            }
        }
        else {
            LOG_ERROR("Can not find firmware data profile.");
        }
    }
}

void MSDEConverterDevice::setNvramDataStreamStopFunc(std::function<void()> stopFunc) {
#if !defined(OS_ARM32) && !defined(OS_MACOS) && !defined(__ANDROID__)
    stopNvramDataFunc_ = stopFunc;
    while(true) {
        {
            if(nvramData_ != nullptr && nvramSize_ != 0 && srcUvcPort_) {
                LOG_INFO("got nvram data succeed.");
                stopGetNvramDataStream();
                // FILE *fp = fopen("./test/nvramData.ccb", "wb");
                // fwrite(nvramData_, 1, nvramSize_, fp);
                // fflush(fp);
                // fclose(fp);
                break;
            }
            else {
                LOG_INFO("got nvram data failed.retrying...");
            }
        }
        ObUtils::sleepMs(300);
    }
#endif
}

void MSDEConverterDevice::depthEngineThreadFunc() {
    // init
    uint8_t retry  = 10;
    bool    status = false;
    while(retry-- > 0 && !status) {
        status = initDepthEngine(profile_);
        ObUtils::sleepMs(300);
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
        processFrameFunc(videoFrame);
    }

    // deinit
    deinitDepthEngine();
}

void MSDEConverterDevice::startDepthEngineThread() {
    terminateDepthEngineThread();
    threadExit_        = false;
    depthEngineThread_ = std::thread(&MSDEConverterDevice::depthEngineThreadFunc, this);
    std::mutex                   mtx;
    std::unique_lock<std::mutex> lk(mtx);
    // LOG_ERROR("===============deinit cv wait");
    deInitCv_.wait(lk);
    // LOG_ERROR("===============deinit cv wait end");
}

void MSDEConverterDevice::terminateDepthEngineThread() {
    threadExit_ = true;
    if(depthEngineThread_.joinable()) {
        {
            std::unique_lock<std::mutex> lock(frameQueueMutex_);
            frameQueueCV_.notify_one();
        }
        depthEngineThread_.join();
    }
}

void MSDEConverterDevice::onNvramDataCallback(VideoFrameObject fo) {
    std::unique_lock<std::recursive_mutex> lk(streamMutex_, std::defer_lock);
    if(lk.try_lock()) {
        ObcFrameHeader *frameHeader = (ObcFrameHeader *)fo.frameData;
        nvramSize_                  = frameHeader->metadataHeader.size;
        if(!nvramData_) {
            nvramData_ = new uint8_t[nvramSize_];
        }
        memcpy(nvramData_, (uint8_t *)fo.frameData + sizeof(ObcFrameHeader), nvramSize_);
    }
}

}  // namespace pal

}  // namespace libobsensor
