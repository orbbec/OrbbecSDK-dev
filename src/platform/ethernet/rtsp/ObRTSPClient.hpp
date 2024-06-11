
#pragma once
#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "ObRTPSink.hpp"
#include <string>
#include <mutex>
#include <condition_variable>

namespace libobsensor {
namespace pal {
const static uint16_t RTSP_RESPONSE_TIMEOUT_MS          = 12000;
const static uint16_t RTSP_RESPONSE_TIMEOUT_TEARDOWN_MS = 2000;
const static bool     RTP_STREAMING_OVER_TCP            = true;

enum CommandState {
    CMD_RESP_WITH_ERROR = -2,
    CMD_TIMEOUT         = -1,
    CMD_WAITING_RESP    = 0,
    CMD_DONE            = 1,
};

enum RTSPState {
    RTSP_DESCRIBE = 0,
    RTSP_SETUP,
    RTSP_PLAY,
    RTSP_TEARDOWN,
};

class ObRTSPClient : public RTSPClient {
public:
    static ObRTSPClient *createNew(UsageEnvironment &env, char const *rtspURL, FrameCallbackUnsafe callback, int verbosityLevel = 0,
                                   char const *applicationName = NULL, portNumBits tunnelOverHTTPPortNum = 0, int socketNumToServer = -1);

    virtual ~ObRTSPClient() noexcept;

public:
    void startStream();
    void stopStream();

protected:
    ObRTSPClient(UsageEnvironment &env, char const *rtspURL, FrameCallbackUnsafe callback, int verbosityLevel = 0, char const *applicationName = NULL,
                 portNumBits tunnelOverHTTPPortNum = 0, int socketNumToServer = -1);

private:
    void DESCRIBE();
    // DESCRIBE命令响应，解析SDP，创建media session
    static void cmdResponseHandlerDESCRIBE(RTSPClient *rtspClient, int resultCode, char *resultString);

    void SETUP();
    void setupNextSubsession();
    // SETUP命令响应，完善Subsession的创建（创建并绑定sink）
    static void cmdResponseHandlerSETUP(RTSPClient *rtspClient, int resultCode, char *resultString);

    void PLAY();
    // PLAY命令响应，如果流是有一定时长的，创建定时任务取关闭流
    static void cmdResponseHandlerPLAY(RTSPClient *rtspClient, int resultCode, char *resultString);

    void TEARDOWN();
    // TEARDOWN 响应
    static void cmdResponseHandlerTEARDOWN(RTSPClient *rtspClient, int resultCode, char *resultString);

    // Other event handler functions:
    static void subsessionAfterPlayingHandler(void *clientData);  // called when a stream's subsession (e.g., audio or video substream) ends
    static void subsessionByeHandler(void *clientData, char const *reason);
    // called when a RTCP "BYE" is received for a subsession
    static void streamStopEventHandler(void *clientData);
    // called at the end of a stream's expected duration (if the stream has not already signaled its end using a RTCP "BYE")

private:
    void clearSinks();

private:
    VideoFrameCallback frameCallback_;

    std::string                 errorMsg_;
    CommandState                commandState_;
    std::recursive_mutex        commandMutex_;
    std::condition_variable_any commandCv_;

    RTSPState RTSPState_;

    MediaSession            *mediaSession_      = nullptr;
    MediaSubsession         *curSubsession_     = nullptr;
    MediaSubsessionIterator *curSubsessionIter_ = nullptr;

    double    duration_;
    TaskToken streamTimerTask_ = NULL;
};
}  // namespace pal
}  // namespace libobsensor
