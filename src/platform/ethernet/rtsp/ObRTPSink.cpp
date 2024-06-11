#include "ObRTPSink.hpp"

#include "H264VideoRTPSource.hh"
#include "logger/Logger.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {
namespace pal {
#define H264_NAL_SPS 7  // Sequence parameter set
#define H264_NAL_PPS 8  // Picture parameter set

#define H265_NAL_VPS 32  // Video Parameter Set
#define H265_NAL_SPS 33  // Sequence parameter set
#define H265_NAL_PPS 34  // Picture parameter set

#pragma pack(2)
typedef struct {
    uint64_t frameCounter;  // 帧序号
    uint16_t extentionLen;  // 扩展数据长度，默认为0
    uint64_t timestamp;     // 时间戳
    uint32_t width;         // 帧像素宽度
    uint32_t height;        // 帧像素高度
} OBNetworkFrameHeader;
#pragma pack()

ObRTPSink *ObRTPSink::createNew(UsageEnvironment &env, MediaSubsession &subsession, FrameCallbackUnsafe callback, char const *streamId) {
    return new ObRTPSink(env, subsession, callback, streamId);
}

ObRTPSink::ObRTPSink(UsageEnvironment &env, MediaSubsession &subsession, FrameCallbackUnsafe callback, char const *streamId)
    : MediaSink(env), subsession_(subsession), frameCallback_(callback), frameCount_(0), destroy_(false), currentFrame_(nullptr) {
    streamId_ = strDup(streamId);
    envir() << "ObRTPSink created! streamId = " << streamId_ << "\n";

    auto frame = std::make_shared<ObRTPFrame>(MAX_FRAME_DATA_SIZE);
    // 将vps/pps/sps 装填到帧头
    if(strcmp(subsession_.codecName(), "H264") == 0) {
        unsigned int  nalUnitNum;
        SPropRecord  *pSPropRecord;
        unsigned char nalUnitStartCode[4] = { 0, 0, 0, 1 };

        pSPropRecord = parseSPropParameterSets(subsession_.fmtp_spropparametersets(), nalUnitNum);
        for(unsigned int i = 0; i < nalUnitNum; i++) {
            memcpy(frame->frameDataBuffer + frame->frameHeaderSize, &nalUnitStartCode[0], 4);
            frame->frameHeaderSize += 4;
            memcpy(frame->frameDataBuffer + frame->frameHeaderSize, pSPropRecord[i].sPropBytes, pSPropRecord[i].sPropLength);
            frame->frameHeaderSize += pSPropRecord[i].sPropLength;
        }
        memcpy(frame->frameDataBuffer + frame->frameHeaderSize, &nalUnitStartCode[0], 4);
        frame->frameHeaderSize += 4;
        delete[] pSPropRecord;
    }
    else if(strcmp(subsession_.codecName(), "H265") == 0) {
        SPropRecord *sPropRecords[3];
        unsigned     numSPropRecords[3];
        sPropRecords[0] = parseSPropParameterSets(subsession_.fmtp_spropvps(), numSPropRecords[0]);
        sPropRecords[1] = parseSPropParameterSets(subsession_.fmtp_spropsps(), numSPropRecords[1]);
        sPropRecords[2] = parseSPropParameterSets(subsession_.fmtp_sproppps(), numSPropRecords[2]);

        unsigned char nalUnitStartCode[4] = { 0, 0, 0, 1 };

        for(uint8_t i = 0; i < 3; i++) {
            for(uint32_t j = 0; j < numSPropRecords[i]; j++) {
                memcpy(frame->frameDataBuffer + frame->frameHeaderSize, &nalUnitStartCode[0], 4);
                frame->frameHeaderSize += 4;
                memcpy(frame->frameDataBuffer + frame->frameHeaderSize, sPropRecords[i][j].sPropBytes, sPropRecords[i][j].sPropLength);
                frame->frameHeaderSize += sPropRecords[i][j].sPropLength;
            }
            delete[] sPropRecords[i];
        }
        memcpy(frame->frameDataBuffer + frame->frameHeaderSize, &nalUnitStartCode[0], 4);
        frame->frameHeaderSize += 4;
    }
    else {
        frame->frameHeaderSize = 0;
    }

    std::unique_lock<std::mutex> lk(reclaimedRTPFrameMutex_);
    for(uint32_t i = 0; i < MAX_FRAME_NUM - 1; i++) {
        auto newFrame = frame->cloneHeader();
        reclaimedRTPFrameQueue_.push(newFrame);
    }
    reclaimedRTPFrameQueue_.push(frame);

    outputFrameThread_ = std::thread(&ObRTPSink::outputFrameFunc, this);
}

ObRTPSink::~ObRTPSink() {
    envir() << "ObRTPSink destructor! streamId = " << streamId_ << "\n";
    frameCallback_ = nullptr;
    destroy_       = true;
    frameAvailableCv_.notify_all();
    delete[] streamId_;
    if(outputFrameThread_.joinable()) {
        outputFrameThread_.join();
    }
}

void ObRTPSink::afterGettingFrame(void *clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime,
                                  unsigned durationInMicroseconds) {
    ObRTPSink *sink = (ObRTPSink *)clientData;
    sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

void ObRTPSink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned /*durationInMicroseconds*/) {
    // uint64_t start  = utils::getNowTimesUs();
    uint64_t curTsp = utils::getNowTimesUs();

    // static uint64_t dataLenCnt_ = 0;
    // static uint64_t frameCnt_   = 0;
    // static uint64_t lastTsp     = 0;
    // static float    fps;
    // static float    dataRate_;

    // dataLenCnt_ += frameSize;
    // frameCnt_ += 1;
    // if(lastTsp != 0 && curTsp - lastTsp >= 1000) {
    //     auto timeDiff = curTsp - lastTsp;
    //     fps           = (double)frameCnt_ / timeDiff * 1000;
    //     frameCnt_     = 0;
    //     dataRate_     = (double)dataLenCnt_ * 8 / 1000 / 1000 / timeDiff * 1000;
    //     dataLenCnt_   = 0;
    //     lastTsp       = curTsp;
    // }
    // if(lastTsp == 0) {
    //     lastTsp = curTsp;
    // }
    // char uSecsStr[7] = { 0 };
    // sprintf(uSecsStr, "%06u", (unsigned)presentationTime.tv_usec);

    // std::string logStr = utils::to_string() << "New frame received: type=" << subsession_.codecName() << ", tsp=" << (int)presentationTime.tv_sec <<
    // "."
    //                                           << uSecsStr << ", framesize=" << frameSize << ", index=" << *(uint32_t *)frame->frameObj->frameData;
    // if(fps != 0) {
    //     logStr = logStr + ", avrFps=" + std::to_string(fps) + ", avrBitRate=" + std::to_string(dataRate_) + "Mbps";
    // }
    // envir() << logStr.c_str();
    do {
        auto &frameObj = currentFrame_->frameObj;
        if(strcmp(subsession_.codecName(), "OB_FMT_Y16") == 0) {
            auto header          = (OBNetworkFrameHeader *)currentFrame_->frameDataBuffer;
            frameObj->index      = header->frameCounter;
            frameObj->systemTime = curTsp;
            frameObj->deviceTime = header->timestamp;
            // LOG_INFO("TimeStamp: {0}(0x{0:8x}), SysTimeStamp: {1}", frameObj->deviceTime, frameObj->systemTime);
            frameObj->format       = OB_FORMAT_Y16;
            frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize - sizeof(OBNetworkFrameHeader);
            frameObj->frameData    = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader);  // Y16格式需要去除网络头
            frameObj->metadataSize = 0;
            frameObj->metadata     = nullptr;
        }
        // todo-lingyi 增加 Y8 、 yuyv格式 、 RVL格式 和 Y10格式
        else if(strcmp(subsession_.codecName(), "OB_FMT_Y8") == 0) {
            auto header            = (OBNetworkFrameHeader *)currentFrame_->frameDataBuffer;
            frameObj->index        = header->frameCounter;
            frameObj->systemTime   = curTsp;
            frameObj->deviceTime   = header->timestamp;
            frameObj->format       = OB_FORMAT_Y8;
            frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize - sizeof(OBNetworkFrameHeader);
            frameObj->frameData    = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader);  // Y8格式需要去除网络头
            frameObj->metadataSize = 0;
            frameObj->metadata     = nullptr;
        }
        else if(strcmp(subsession_.codecName(), "OB_FMT_Y10") == 0) {
            auto header            = (OBNetworkFrameHeader *)currentFrame_->frameDataBuffer;
            frameObj->index        = header->frameCounter;
            frameObj->systemTime   = curTsp;
            frameObj->deviceTime   = header->timestamp;
            frameObj->format       = OB_FORMAT_Y10;
            frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize - sizeof(OBNetworkFrameHeader);
            frameObj->frameData    = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader);  // Y10格式需要去除网络头
            frameObj->metadataSize = 0;
            frameObj->metadata     = nullptr;
        }
        else if(strcmp(subsession_.codecName(), "OB_FMT_RVL") == 0) {
            auto header            = (OBNetworkFrameHeader *)currentFrame_->frameDataBuffer;
            frameObj->index        = header->frameCounter;
            frameObj->systemTime   = curTsp;
            frameObj->deviceTime   = header->timestamp;
            frameObj->format       = OB_FORMAT_RVL;
            frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize - sizeof(OBNetworkFrameHeader);
            frameObj->frameData    = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader);  // RVL格式需要去除网络头
            frameObj->metadataSize = 0;
            frameObj->metadata     = nullptr;
        }
        else if(strcmp(subsession_.codecName(), "YUYV") == 0) {
            auto header          = (OBNetworkFrameHeader *)currentFrame_->frameDataBuffer;
            frameObj->index      = header->frameCounter;
            frameObj->systemTime = curTsp;
            frameObj->deviceTime = header->timestamp;
            // LOG_INFO("TimeStamp: {0}(0x{0:8x}), SysTimeStamp: {1}", frameObj->deviceTime, frameObj->systemTime);
            frameObj->format       = OB_FORMAT_YUYV;
            frameObj->frameSize    = frameSize - sizeof(OBNetworkFrameHeader) - header->extentionLen;
            frameObj->frameData    = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader) + header->extentionLen;  // YUYV格式需要去除网络头
            frameObj->metadataSize = header->extentionLen;
            frameObj->metadata     = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader);
        }
        else if(strcmp(subsession_.codecName(), "H264") == 0) {
            // frame->frameHeaderSize 其实是frame->frameObj->frameData有效数据的起始地址
            if((currentFrame_->frameDataBuffer[currentFrame_->frameHeaderSize] & 0x1f) == H264_NAL_SPS
               || (currentFrame_->frameDataBuffer[currentFrame_->frameHeaderSize] & 0x1f) == H264_NAL_PPS) {  // SPS(7) or PPS(8)
                break;
            }
            else {
                frameObj->index      = frameCount_;
                frameObj->systemTime = curTsp;
                // frameObj->deviceTime   = (uint64_t)presentationTime.tv_sec * 1000000 + (int)presentationTime.tv_usec;
                // frameObj->deviceTime   = ((RTPSource *)fSource)->curPacketRTPTimestamp();
                frameObj->deviceTime   = subsession_.getNormalPlayTime(presentationTime);
                frameObj->format       = OB_FORMAT_H264;
                frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize;
                frameObj->frameData    = currentFrame_->frameDataBuffer;
                frameObj->metadataSize = 0;
                frameObj->metadata     = nullptr;
            }
        }
        else if(strcmp(subsession_.codecName(), "H265") == 0) {
            // frame->frameHeaderSize 其实是frame->frameObj->frameData有效数据的起始地址
            if(((currentFrame_->frameDataBuffer[currentFrame_->frameHeaderSize] & 0x7f) >> 1) == H265_NAL_VPS
               || ((currentFrame_->frameDataBuffer[currentFrame_->frameHeaderSize] & 0x7f) >> 1) == H265_NAL_SPS
               || ((currentFrame_->frameDataBuffer[currentFrame_->frameHeaderSize] & 0x7f) >> 1) == H265_NAL_PPS) {
                break;
            }
            else {
                frameObj->index      = frameCount_;
                frameObj->systemTime = curTsp;
                // frameObj->deviceTime   = (uint64_t)presentationTime.tv_sec * 1000000 + (int)presentationTime.tv_usec;
                // frameObj->deviceTime   = ((RTPSource *)fSource)->curPacketRTPTimestamp();
                frameObj->deviceTime   = subsession_.getNormalPlayTime(presentationTime);
                frameObj->format       = OB_FORMAT_H265;
                frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize;
                frameObj->frameData    = currentFrame_->frameDataBuffer;
                frameObj->metadataSize = 0;
                frameObj->metadata     = nullptr;
            }
        }
        else if(strcmp(subsession_.codecName(), "JPEG") == 0) {
            frameObj->index      = frameCount_;
            frameObj->systemTime = curTsp;
            // frameObj->deviceTime   = (uint64_t)presentationTime.tv_sec * 1000000 + (int)presentationTime.tv_usec;
            // frameObj->deviceTime   = ((RTPSource *)fSource)->curPacketRTPTimestamp();
            frameObj->deviceTime = subsession_.getNormalPlayTime(presentationTime);
            // LOG_INFO("TimeStamp: {0}(0x{0:8x}), SysTimeStamp: {1}", frameObj->deviceTime, frameObj->systemTime);
            frameObj->format    = OB_FORMAT_MJPG;
            frameObj->frameSize = frameSize + currentFrame_->frameHeaderSize;
            // 单机任意触发彩色，存在长度556包非帧包，调用帧回调 @LingYi
            // if(frameObj->frameSize == 556) 跳过次包
            frameObj->frameData    = currentFrame_->frameDataBuffer;
            frameObj->metadataSize = 0;
            frameObj->metadata     = nullptr;
        }
        else if(strcmp(subsession_.codecName(), "OB_FMT_MJPEG") == 0) {
            auto header            = (OBNetworkFrameHeader *)currentFrame_->frameDataBuffer;
            frameObj->index        = frameCount_;
            frameObj->systemTime   = curTsp;
            frameObj->deviceTime   = header->timestamp;
            frameObj->format       = OB_FORMAT_MJPG;
            frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize - sizeof(OBNetworkFrameHeader);
            frameObj->frameData    = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader);
            frameObj->metadataSize = 0;
            frameObj->metadata     = nullptr;
        }
        else if(strcmp(subsession_.codecName(), "OB_FMT_Y8") == 0) {
            auto header            = (OBNetworkFrameHeader *)currentFrame_->frameDataBuffer;
            frameObj->index        = header->frameCounter;
            frameObj->systemTime   = curTsp;
            frameObj->deviceTime   = header->timestamp;
            frameObj->format       = OB_FORMAT_Y8;
            frameObj->frameSize    = frameSize + currentFrame_->frameHeaderSize - sizeof(OBNetworkFrameHeader);
            frameObj->frameData    = currentFrame_->frameDataBuffer + sizeof(OBNetworkFrameHeader);
            frameObj->metadataSize = 0;
            frameObj->metadata     = nullptr;
        }
        else {
            // VideoFrameObject fo = { frameSize, 0,      frame->frameObj->frameData,
            //                    nullptr,   curTsp, (uint64_t)presentationTime.tv_sec * 1000 + (int)presentationTime.tv_usec / 1000 };
            break;
        }

        {
            std::unique_lock<std::mutex> lk(outputRTPFrameQueueMutex_);
            outputRTPFrameQueue_.push(currentFrame_);
            currentFrame_.reset();
        }
        frameAvailableCv_.notify_all();
        frameCount_++;
    } while(0);

    // Then continue, to request the next frame of data:
    continuePlaying();

    // auto dur = utils::getNowTimesUs() - start;
    // if(dur > 1000) {
    //     LOG_ERROR("Live555 afterGettingFrame dur=" << dur << "usec > 1000 usec";
    // }
    // else if(dur > 100) {
    //     LOG_WARN("Live555 afterGettingFrame dur=" << dur << "usec > 100 usec";
    // }
}

Boolean ObRTPSink::continuePlaying() {
    if(fSource == NULL || frameCallback_ == NULL) {
        return False;
    }

    if(!currentFrame_) {
        std::unique_lock<std::mutex> lk(reclaimedRTPFrameMutex_);
        if(!reclaimedRTPFrameQueue_.empty()) {
            currentFrame_ = reclaimedRTPFrameQueue_.front();
            reclaimedRTPFrameQueue_.pop();
        }
    }

    if(!currentFrame_) {
        // 主动丢帧
        std::unique_lock<std::mutex> lk(outputRTPFrameQueueMutex_);
        if(!outputRTPFrameQueue_.empty()) {
            currentFrame_ = outputRTPFrameQueue_.front();
            outputRTPFrameQueue_.pop();
            LOG_WARN("Drop output-frame to receive new frame due to reclaimed-frame queue is empty: frameIndex={0}, devTsp={1}", currentFrame_->frameObj->index,
                     currentFrame_->frameObj->deviceTime);
        }
    }

    // Request the next frame of data from our input source.  "afterGettingFrame()" will get called later, when it arrives:
    fSource->getNextFrame(currentFrame_->frameDataBuffer + currentFrame_->frameHeaderSize, MAX_FRAME_DATA_SIZE - currentFrame_->frameHeaderSize,
                          afterGettingFrame, this, onSourceClosure, this);
    return true;
}

void ObRTPSink::outputFrameFunc() {
    while(!destroy_) {
        std::shared_ptr<ObRTPFrame> outputFrame;
        {
            std::unique_lock<std::mutex> lk(outputRTPFrameQueueMutex_);
            frameAvailableCv_.wait(lk, [&]() { return !outputRTPFrameQueue_.empty() || destroy_; });
            if(destroy_) {
                break;
            }
            outputFrame = outputRTPFrameQueue_.front();
            outputRTPFrameQueue_.pop();
        }

        if(outputFrame) {
            frameCallback_(*outputFrame->frameObj);
            {
                std::unique_lock<std::mutex> lk(reclaimedRTPFrameMutex_);
                reclaimedRTPFrameQueue_.push(outputFrame);
            }
        }
    }
}

}  // namespace pal
}  // namespace libobsensor