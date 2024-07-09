// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.
#pragma once

#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <map>

#include "utils_types.h"

namespace ob_smpl {

// arrange type
typedef enum {
    ARRANGE_SINGLE,      // Only show the first frame
    ARRANGE_ONE_ROW,     // Arrange the frames in the array as a row
    ARRANGE_ONE_COLUMN,  // Arrange the frames in the array as a column
    ARRANGE_GRID,        // Arrange the frames in the array as a grid
    ARRANGE_OVERLAY      // Overlay the first two frames in the array
} ArrangeType;

class CVWindow {
public:
    // create a window with the specified name, width and height
    CVWindow(std::string name, uint32_t width = 1280, uint32_t height = 720, ArrangeType arrangeType = ARRANGE_SINGLE);
    ~CVWindow() noexcept;

    // run the window loop
    bool run();

    // close window
    void close();

    // clear cached frames and mats
    void reset();

    // add frames to view
    void pushFramesToView(std::vector<std::shared_ptr<const ob::Frame>> frames, int groupId = 0);
    void pushFramesToView(std::shared_ptr<const ob::Frame> currentFrame, int groupId = 0);

    // wait for the key to be pressed
    int waitKey(uint32_t timeoutMsec = 1);

    // set show frame info
    void setShowInfo(bool show);

    // set alpha, only valid when arrangeType_ is ARRANGE_OVERLAY
    void setAlpha(float alpha);

    // set the window size
    void resize(int width, int height);

    // set the key prompt
    void setKeyPrompt(const std::string &prompt);

    // set the log message
    void addLog(const std::string &log);

private:
    // frames processing thread function
    void processFrames();

    // arrange frames in the renderMat_ according to the arrangeType_
    void arrangeFrames();

    // add info to mat
    static cv::Mat visualize(std::shared_ptr<const ob::Frame> frame);

    // draw info to mat
    static void drawInfo(cv::Mat &imageMat, std::shared_ptr<const ob::VideoFrame> &frame);

    static cv::Mat resizeMatKeepAspectRatio(const cv::Mat &mat, int width, int height);

private:
    std::string name_;
    ArrangeType arrangeType_;
    uint32_t    width_;
    uint32_t    height_;
    bool        closed_;
    bool        showInfo_;
    float       alpha_;

    int                     key_;
    std::mutex              keyMtx_;
    std::condition_variable keyCv_;

    std::thread                                                  processThread_;
    std::map<int, std::vector<std::shared_ptr<const ob::Frame>>> srcFrameGroups_;
    std::mutex                                                   srcFrameGroupsMtx_;
    std::condition_variable                                      srcFrameGroupsCv_;

    using StreamsMatMap = std::map<int, std::pair<std::shared_ptr<const ob::Frame>, cv::Mat>>;
    StreamsMatMap mapGroups_;
    std::mutex    renderMatsMtx_;
    cv::Mat       renderMat_;

    std::string prompt_;
    bool        showPrompt_;
    uint64      winCreatedTime_;

    std::string log_;
    uint64      logCreatedTime_;
};

}  // namespace ob_smpl