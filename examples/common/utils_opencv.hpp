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

typedef enum {
    RENDER_SINGLE,      // only render the first frame in the array
    RENDER_ONE_ROW,     // Render the frames in the array as a row
    RENDER_ONE_COLUMN,  // render the frames in the array as a column
    RENDER_GRID,        // Render the frames in the array as a grid
    RENDER_OVERLAY      // Render the frames in the array as an overlay
} RenderType;

class CVWindow {
public:
    // create a window with the specified name, width and height
    CVWindow(std::string name, uint32_t width, uint32_t height, RenderType renderType_ = RENDER_SINGLE);
    ~CVWindow() noexcept;

    // run the window loop
    bool run();

    // close window
    void close();

    // clear cached frames and mats
    void reset();

    // add frames to the rendering
    void pushFramesToShow(std::vector<std::shared_ptr<const ob::Frame>> frames, int groupId = 0);
    void pushFramesToShow(std::shared_ptr<const ob::Frame> currentFrame, int groupId = 0);

    // wait for the key to be pressed
    int waitKey(uint32_t timeoutMsec = 1);

    // set show frame info
    void setShowInfo(bool show);

    // set alpha, only valid when renderType_ is RENDER_OVERLAY
    void setAlpha(float alpha);

    // set the window size
    void resize(int width, int height);

private:
    // frames processing thread function
    void processFrames();
    void renderImages();

    // add info to mat
    static cv::Mat visualize(std::shared_ptr<const ob::Frame> frame);
    static void    drawInfo(cv::Mat &imageMat, std::shared_ptr<const ob::VideoFrame> &frame);

private:
    std::string name_;
    RenderType  renderType_;
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
};

}  // namespace ob_smpl