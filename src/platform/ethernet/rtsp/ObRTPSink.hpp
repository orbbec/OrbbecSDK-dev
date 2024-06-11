#pragma once
#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "ISourcePort.hpp"

#include <queue>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace libobsensor {


struct ObRTPFrame {
    ObRTPFrame(uint32_t maxDataSize) : maxBuffSize(maxDataSize) {
        frameObj        = new VideoFrameObject();
        frameDataBuffer = new uint8_t[maxDataSize];
        frameHeaderSize = 0;
    }

    ~ObRTPFrame() {
        delete[] frameDataBuffer;
        delete frameObj;
    }

    std::shared_ptr<ObRTPFrame> cloneHeader() {
        auto rstFrame = std::make_shared<ObRTPFrame>(maxBuffSize);
        memcpy(rstFrame->frameDataBuffer, frameDataBuffer, frameHeaderSize);
        rstFrame->frameHeaderSize = frameHeaderSize;
        return rstFrame;
    }

    VideoFrameObject *frameObj;
    uint8_t          *frameDataBuffer;
    uint32_t          frameHeaderSize;  // h264&h265的帧头可以直接解析SDP或得，后续接受的数据要放在帧头之后
    uint32_t          maxBuffSize;
};

class ObRTPSink : public MediaSink {
public:
    static ObRTPSink *createNew(UsageEnvironment  &env,
                                MediaSubsession   &subsession,  // identifies the kind of data that's being received
                                FrameCallbackUnsafe callback,
                                char const        *streamId = NULL);  // identifies the stream itself (optional)

    virtual ~ObRTPSink() noexcept;

protected:
    ObRTPSink(UsageEnvironment &env, MediaSubsession &subsession, FrameCallbackUnsafe callback, char const *streamId = NULL);

private:
    static void afterGettingFrame(void *clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime,
                                  unsigned durationInMicroseconds);
    void        afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds);

private:
    // redefined virtual functions:
    virtual Boolean continuePlaying();
    void            outputFrameFunc();

private:
    MediaSubsession   &subsession_;
    char              *streamId_;
    VideoFrameCallback frameCallback_;
    uint64_t           frameCount_;
    std::atomic<bool>  destroy_;

    std::queue<std::shared_ptr<ObRTPFrame>> reclaimedRTPFrameQueue_;
    std::mutex                              reclaimedRTPFrameMutex_;
    std::queue<std::shared_ptr<ObRTPFrame>> outputRTPFrameQueue_;
    std::mutex                              outputRTPFrameQueueMutex_;
    std::condition_variable                 frameAvailableCv_;
    std::shared_ptr<ObRTPFrame>             currentFrame_;

    const uint8_t  MAX_FRAME_NUM       = 4;
    const uint32_t MAX_FRAME_DATA_SIZE = 3 * 3840 * 2160;

    std::thread outputFrameThread_;
};


}  // namespace libobsensor
