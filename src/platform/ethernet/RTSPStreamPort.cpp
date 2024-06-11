#include "RTSPStreamPort.hpp"
#include "logger/Logger.hpp"
#include "exception/ObException.hpp"
#include <map>
namespace libobsensor {
namespace pal {
std::string mapFormatToString(OBFormat format) {
    static std::map<OBFormat, std::string> formatToStringMap = {
        { OB_FORMAT_YUYV, "yuyv" },     { OB_FORMAT_YUY2, "yuyv" }, { OB_FORMAT_UYVY, "uyvy" }, { OB_FORMAT_NV12, "nv12" }, { OB_FORMAT_NV21, "nv21" },
        { OB_FORMAT_MJPG, "mjpg" },     { OB_FORMAT_H264, "h264" }, { OB_FORMAT_H265, "h265" }, { OB_FORMAT_Y16, "y16" },   { OB_FORMAT_Y8, "y8" },
        { OB_FORMAT_Y10, "y10" },       { OB_FORMAT_Y11, "y11" },   { OB_FORMAT_Y12, "y12" },   { OB_FORMAT_GRAY, "y8" },   { OB_FORMAT_HEVC, "h265" },
        { OB_FORMAT_I420, "i420" },     { OB_FORMAT_RLE, "rle" },   { OB_FORMAT_RGB, "rgb" },   { OB_FORMAT_BGR, "bgr" },   { OB_FORMAT_Y14, "y14" },
        { OB_FORMAT_BGRA, "bgra" },     { OB_FORMAT_RGBA, "rgba" }, { OB_FORMAT_RVL, "rvl" },   { OB_FORMAT_BYR2, "byr2" }, { OB_FORMAT_RW16, "rw16" },
        { OB_FORMAT_DISP16, "disp16" },
    };
    return formatToStringMap[format];
}

std::string mapStreamTypeToString(OBStreamType streamType) {
    static std::map<OBStreamType, std::string> streamTypeToStringMap = {
        { OB_STREAM_IR, "ir" },           { OB_STREAM_DEPTH, "depth" },       { OB_STREAM_COLOR, "color" },
        { OB_STREAM_IR_LEFT, "ir_left" }, { OB_STREAM_IR_RIGHT, "ir_right" },
    };
    return streamTypeToStringMap[streamType];
}

RTSPStreamPort::RTSPStreamPort(std::shared_ptr<const RTSPStreamPortInfo> portInfo)
    : portInfo_(portInfo), destroy_(0), currentRtspClient_(nullptr), streamStarted_(false) {
    taskScheduler_ = BasicTaskScheduler::createNew();
    live555Env_    = ObUsageEnvironment::createNew(*taskScheduler_);
}

RTSPStreamPort::~RTSPStreamPort() {

    // 1. 关流, 关闭rtsp客户端
    TRY_EXECUTE(stopAllStream());  // try stop all stream

    // 2. 释放调度器
    delete taskScheduler_;
    taskScheduler_ = NULL;

    // 3. live555 env 释放
    live555Env_->reclaim();
    live555Env_ = NULL;
}

std::vector<std::shared_ptr<const VideoStreamProfile>> RTSPStreamPort::getStreamProfileList() {
    if(streamProfileList_.empty()) {
        // todo: std::vector<std::shared_ptr<const VideoStreamProfile>> 从设备获取
        if(portInfo_->streamType == OB_STREAM_DEPTH) {
            // todo-lingyi 增加了注释
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1024, 1024, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1024, 1024, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 30));

            // todo-lingyi 增加网络模式加载基础分辨率列表 Gemini2XL
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 400, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1280, 800, 10));

            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_RVL, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_RVL, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_RVL, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_RVL, 640, 400, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_RVL, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_RVL, 1280, 800, 10));
        }
        else if(portInfo_->streamType == OB_STREAM_COLOR) {
            // todo-lingyi 增加网络模式加载基础分辨率列表 Gemini2XL
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 800, 600, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 800, 600, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 800, 600, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 800, 600, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 360, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 360, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 360, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 360, 20));

            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 1280, 800, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 360, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 360, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 360, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 360, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 640, 400, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 800, 600, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 800, 600, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 800, 600, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 800, 600, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 1280, 720, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_YUYV, 1280, 720, 10));

            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 3840, 2160, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 3840, 2160, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 3840, 2160, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 2560, 1440, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 2560, 1440, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 2560, 1440, 25));
            //// streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 2560, 1440, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1920, 1080, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1920, 1080, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1920, 1080, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1920, 1080, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 720, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 960, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 960, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 960, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 960, 30));

            // todo-lingyi 增加了注释
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 3840, 2160, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 3840, 2160, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 3840, 2160, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 2560, 1440, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 2560, 1440, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 2560, 1440, 25));
            //// streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 2560, 1440, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1920, 1080, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1920, 1080, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1920, 1080, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1920, 1080, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 720, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 720, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 720, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 720, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 960, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 960, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 960, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H264, 1280, 960, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 3840, 2160, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 3840, 2160, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 3840, 2160, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 2560, 1440, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 2560, 1440, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 2560, 1440, 25));
            //// streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 2560, 1440, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1920, 1080, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1920, 1080, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1920, 1080, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1920, 1080, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 720, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 720, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 720, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 720, 30));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 960, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 960, 15));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 960, 25));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_H265, 1280, 960, 30));
        }
        else if(portInfo_->streamType == OB_STREAM_IR) {
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1024, 1024, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1024, 1024, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1024, 1024, 25));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 1024, 1024, 30));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 25));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 512, 512, 30));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 25));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 640, 576, 30));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 25));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y16, 320, 288, 30));
        }
        // todo-lingyi fGemini2XL增加左右IR
        else if(portInfo_->streamType == OB_STREAM_IR_LEFT) {
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 20));

            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 20));

            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 1280, 800, 10));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 640, 400, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 640, 400, 10));
        }
        else if(portInfo_->streamType == OB_STREAM_IR_RIGHT) {
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 1280, 800, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y8, 640, 400, 20));

            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 1280, 800, 20));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 10));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 15));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_MJPG, 640, 400, 20));

            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 1280, 800, 5));
            streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 1280, 800, 10));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 640, 400, 5));
            // streamProfileList_.emplace_back(std::make_shared<VideoStreamProfile>(portInfo_->streamType, OB_FORMAT_Y10, 640, 400, 10));
        }
    }
    return streamProfileList_;
}

void RTSPStreamPort::startStream(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) {
    if(streamStarted_) {
        LOG_WARN("Stream already started!");
    }
    else {
        BEGIN_TRY_EXECUTE({
            createClient(profile, callback);
            currentRtspClient_->startStream();
        })
        CATCH_EXCEPTION_AND_EXECUTE({
            closeClient();
            throw;
        })
        streamStarted_ = true;
        LOG_DEBUG("Stream started!");
    }
}
void RTSPStreamPort::stopStream(std::shared_ptr<const VideoStreamProfile> profile) {
    // 当前RTSPStreamPort只支持一路数据流
    // todo: 增加profile与currentStreamProfile_校验
    stopStream();
}

void RTSPStreamPort::stopAllStream() {
    // 当前RTSPStreamPort只支持一路数据流
    stopStream();
}

void RTSPStreamPort::stopStream() {
    if(streamStarted_) {
        BEGIN_TRY_EXECUTE({ currentRtspClient_->stopStream(); })
        CATCH_EXCEPTION_AND_EXECUTE({
            closeClient();
            streamStarted_ = false;
            throw;
        })
        closeClient();
        streamStarted_ = false;
        LOG_DEBUG("Stream stoped!");
    }
    else {
        LOG_WARN("Stream have not been started!");
    }
}

void RTSPStreamPort::createClient(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) {
    destroy_         = 0;
    eventLoopThread_ = std::thread([&]() { live555Env_->taskScheduler().doEventLoop(&destroy_); });

    currentStreamProfile_ = std::dynamic_pointer_cast<const VideoStreamProfile>(profile);
    std::string formatStr = std::to_string(currentStreamProfile_->width) + "_" + std::to_string(currentStreamProfile_->height) + "_"
                            + std::to_string(currentStreamProfile_->fps) + "_" + mapFormatToString(currentStreamProfile_->format);
    std::string url = std::string("rtsp://") + portInfo_->address + ":" + std::to_string(portInfo_->port) + "/"
                      + mapStreamTypeToString(currentStreamProfile_->streamType) + "/" + formatStr;
    currentRtspClient_ = ObRTSPClient::createNew(*live555Env_, url.c_str(), callback, 1, "OrbbecSDK");
    LOG_DEBUG("ObRTSPClient created! url={}", url);
}

void RTSPStreamPort::closeClient() {
    destroy_ = 1;
    if(eventLoopThread_.joinable()) {
        eventLoopThread_.join();
    }
    std::string url = "";
    if(currentRtspClient_) {
        url = currentRtspClient_->url();
        // 需要在client关闭前关闭事务线程，否则会偶发crash
        Medium::close(currentRtspClient_);
        currentRtspClient_ = nullptr;
    }
    LOG_DEBUG("ObRTSPClient close!  url={}", url);
}

std::shared_ptr<const SourcePortInfo> RTSPStreamPort::getSourcePortInfo() const {
    return portInfo_;
}

}  // namespace pal
}  // namespace libobsensor
