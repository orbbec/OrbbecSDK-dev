#include "utils_opencv.hpp"
#include "utils_types.h"

namespace ob_smpl {

CVWindow::CVWindow(std::string name, uint32_t width, uint32_t height, RenderType renderType_)
    : name_(std::move(name)), renderType_(renderType_), width_(width), height_(height), closed_(false), showInfo_(true), alpha_(0.6f), key_(-1) {

    renderMat_ = cv::Mat::zeros(height_, width_, CV_8UC3);
    cv::putText(renderMat_, "Waiting for streams...", cv::Point(8, 16), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    cv::imshow(name_, renderMat_);

    // start processing thread
    processThread_ = std::thread(&CVWindow::processFrames, this);

    cv::namedWindow(name_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(name_, width_, height_);
}

CVWindow::~CVWindow() {
    close();
    cv::destroyWindow(name_);
}

// if window is closed
bool CVWindow::run() {

    {
        // show render mat
        std::lock_guard<std::mutex> lock(renderMatsMtx_);
        cv::imshow(name_, renderMat_);
    }

    int key = cv::waitKey(1);
    if(key != -1) {
        std::unique_lock<std::mutex> lk(keyMtx_);
        key_ = key;
        keyCv_.notify_all();

        if(key == ESC_KEY) {
            closed_ = true;
            srcFrameGroupsCv_.notify_all();
        }
    }

    return !closed_;
}

// close window
void CVWindow::close() {
    {
        std::lock_guard<std::mutex> lock(renderMatsMtx_);
        mapGroups_.clear();
        srcFrameGroups_.clear();
        srcFrameGroupsCv_.notify_all();
    }

    closed_ = true;

    if(processThread_.joinable()) {
        processThread_.join();
    }
}

void CVWindow::reset() {
    // close thread and clear cache
    close();

    // restart thread
    closed_        = false;
    processThread_ = std::thread(&CVWindow::processFrames, this);
}

// set the window size
void CVWindow::resize(int width, int height) {
    width_  = width;
    height_ = height;
    cv::resizeWindow(name_, width_, height_);
}

// add frames to the show
void CVWindow::pushFramesToShow(std::vector<std::shared_ptr<const ob::Frame>> frames, int groupId) {
    if(frames.empty()) {
        return;
    }

    std::vector<std::shared_ptr<const ob::Frame>> singleFrames;
    for(auto &frame: frames) {
        if(frame == nullptr) {
            continue;
        }

        if(!frame->is<ob::FrameSet>()) {
            // single frame, add to the list
            singleFrames.push_back(frame);
            continue;
        }

        // FrameSet contains multiple frames
        auto frameSet = frame->as<ob::FrameSet>();
        for(uint32_t index = 0; index < frameSet->getCount(); index++) {
            auto subFrame = frameSet->getFrameByIndex(index);
            singleFrames.push_back(subFrame);
        }
    }

    std::lock_guard<std::mutex> lk(srcFrameGroupsMtx_);
    srcFrameGroups_[groupId] = singleFrames;
    srcFrameGroupsCv_.notify_one();
}

void CVWindow::pushFramesToShow(std::shared_ptr<const ob::Frame> currentFrame, int groupId) {
    pushFramesToShow(std::vector<std::shared_ptr<const ob::Frame>>{ currentFrame }, groupId);
}

// wait for the key to be pressed
int CVWindow::waitKey(uint32_t timeoutMsec) {
    std::unique_lock<std::mutex> lk(keyMtx_);
    keyCv_.wait_for(lk, std::chrono::milliseconds(timeoutMsec), [&] { return key_ != -1; });
    int key = key_;
    key_    = -1;
    return key;
}

// set show frame info
void CVWindow::setShowInfo(bool show) {
    showInfo_ = show;
}

// set alpha for OVERLAY render mode
void CVWindow::setAlpha(float alpha) {
    alpha_ = alpha;
    if(alpha_ < 0) {
        alpha_ = 0;
    }
    else if(alpha_ > 1) {
        alpha_ = 1;
    }
}

// frames processing thread
void CVWindow::processFrames() {
    std::map<int, std::vector<std::shared_ptr<const ob::Frame>>> frameGroups;
    while(!closed_) {
        {
            std::unique_lock<std::mutex> lk(srcFrameGroupsMtx_);
            srcFrameGroupsCv_.wait(lk, [this] { return !srcFrameGroups_.empty() || closed_; });
            if(closed_) {
                break;
            }
            frameGroups = srcFrameGroups_;
        }

        if(frameGroups.empty()) {
            continue;
        }

        for(const auto &framesItem: frameGroups) {
            int         groupId = framesItem.first;
            const auto &frames  = framesItem.second;
            for(const auto &frame: frames) {
                auto rstMat = visualize(frame);
                if(!rstMat.empty()) {
                    int uid         = groupId * OB_FRAME_TYPE_COUNT + static_cast<int>(frame->getType());
                    mapGroups_[uid] = { frame, rstMat };
                }
            }
        }

        if(mapGroups_.empty()) {
            continue;
        }

        renderImages();
    }
}

void CVWindow::renderImages() {
    cv::Mat renderMat;
    try {
        if(renderType_ == ob_smpl::RENDER_SINGLE) {
            auto &mat = mapGroups_.begin()->second.second;
            cv::resize(mat, renderMat, cv::Size(static_cast<int>(width_), static_cast<int>(height_)));
        }
        else if(renderType_ == ob_smpl::RENDER_ONE_ROW) {
            for(auto &item: mapGroups_) {
                auto   &mat = item.second.second;
                cv::Mat resizeMat;
                cv::resize(mat, resizeMat, cv::Size(static_cast<int>(width_ / mapGroups_.size()), height_));
                if(renderMat.dims > 0 && renderMat.cols > 0 && renderMat.rows > 0) {
                    cv::hconcat(renderMat, resizeMat, renderMat);
                }
                else {
                    renderMat = resizeMat;
                }
            }
        }
        else if(renderType_ == ob_smpl::RENDER_ONE_COLUMN) {
            for(auto &item: mapGroups_) {
                auto   &mat = item.second.second;
                cv::Mat resizeMat;
                cv::resize(mat, resizeMat, cv::Size(static_cast<int>(width_), static_cast<int>(height_ / mapGroups_.size())));
                if(renderMat.dims > 0 && renderMat.cols > 0 && renderMat.rows > 0) {
                    cv::vconcat(renderMat, resizeMat, renderMat);
                }
                else {
                    renderMat = resizeMat;
                }
            }
        }
        else if(renderType_ == ob_smpl::RENDER_GRID) {
            int count     = static_cast<int>(mapGroups_.size());
            int idealSide = static_cast<int>(std::sqrt(count));
            int rows      = idealSide;
            int cols      = idealSide;
            while(rows * cols < count) {  // find the best row and column count
                cols++;
                if(rows * cols < count) {
                    rows++;
                }
            }

            std::vector<cv::Mat> gridImages;  // store all images in grid
            auto                 it = mapGroups_.begin();
            for(int i = 0; i < rows; i++) {
                std::vector<cv::Mat> rowImages;  // store images in the same row
                for(int j = 0; j < cols; j++) {
                    int     index = i * cols + j;
                    cv::Mat resizeMat;
                    if(index < count) {
                        auto mat = it->second.second;
                        cv::resize(mat, resizeMat, cv::Size(width_ / cols, height_ / rows));
                        it++;
                    }
                    else {
                        resizeMat = cv::Mat::zeros(height_ / rows, width_ / cols, CV_8UC3);  // fill with black
                    }
                    rowImages.push_back(resizeMat);
                }
                cv::Mat lineMat;
                cv::hconcat(rowImages, lineMat);  // horizontal concat all images in the same row
                gridImages.push_back(lineMat);
            }

            cv::vconcat(gridImages, renderMat);  // vertical concat all images in the grid
        }
        else if(renderType_ == ob_smpl::RENDER_OVERLAY && mapGroups_.size() >= 2) {
            cv::Mat     overlayMat;
            const auto &mat1 = mapGroups_.begin()->second.second;
            const auto &mat2 = (mapGroups_.begin()++)->second.second;
            cv::resize(mat1, renderMat, cv::Size(width_, height_));
            cv::resize(mat2, overlayMat, cv::Size(width_, height_));
            float alpha = alpha_;
            for(int i = 0; i < renderMat.rows; i++) {
                for(int j = 0; j < renderMat.cols; j++) {
                    cv::Vec3b &outRgb     = renderMat.at<cv::Vec3b>(i, j);
                    cv::Vec3b &overlayRgb = overlayMat.at<cv::Vec3b>(i, j);

                    outRgb[0] = (uint8_t)(outRgb[0] * (1.0f - alpha) + overlayRgb[0] * alpha);
                    outRgb[1] = (uint8_t)(outRgb[1] * (1.0f - alpha) + overlayRgb[1] * alpha);
                    outRgb[2] = (uint8_t)(outRgb[2] * (1.0f - alpha) + overlayRgb[2] * alpha);
                }
            }
        }
    }
    catch(std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    if(renderMat.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(renderMatsMtx_);
    renderMat_ = renderMat;
}

cv::Mat CVWindow::visualize(std::shared_ptr<const ob::Frame> frame) {
    if(frame == nullptr) {
        return cv::Mat();
    }

    cv::Mat rstMat;
    if(frame->getType() == OB_FRAME_COLOR) {
        auto videoFrame = frame->as<const ob::VideoFrame>();
        switch(videoFrame->getFormat()) {
        case OB_FORMAT_MJPG: {
            cv::Mat rawMat(1, videoFrame->getDataSize(), CV_8UC1, videoFrame->getData());
            rstMat = cv::imdecode(rawMat, 1);
        } break;
        case OB_FORMAT_NV21: {
            cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
        } break;
        case OB_FORMAT_YUYV:
        case OB_FORMAT_YUY2: {
            cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC2, videoFrame->getData());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
        } break;
        case OB_FORMAT_RGB: {
            cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC3, videoFrame->getData());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
        } break;
        case OB_FORMAT_RGBA: {
            cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC4, videoFrame->getData());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_RGBA2BGRA);
        } break;
        case OB_FORMAT_BGRA: {
            rstMat = cv::Mat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC4, videoFrame->getData());
        } break;
        case OB_FORMAT_UYVY: {
            cv::Mat rawMat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC2, videoFrame->getData());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
        } break;
        case OB_FORMAT_I420: {
            cv::Mat rawMat(videoFrame->getHeight() * 3 / 2, videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_I420);
        } break;
        default:
            break;
        }
    }
    else if(frame->getType() == OB_FRAME_DEPTH) {
        auto videoFrame = frame->as<const ob::VideoFrame>();
        if(videoFrame->getFormat() == OB_FORMAT_Y16 || videoFrame->getFormat() == OB_FORMAT_Z16) {
            cv::Mat cvtMat;
            cv::Mat rawMat = cv::Mat(videoFrame->getHeight(), videoFrame->getWidth(), CV_16UC1, videoFrame->getData());
            // depth frame pixel value multiply scale to get distance in millimeter
            float scale = videoFrame->as<ob::DepthFrame>()->getValueScale();

            // threshold to 5.12m
            cv::threshold(rawMat, cvtMat, 5120.0f / scale, 0, cv::THRESH_TRUNC);
            cvtMat.convertTo(cvtMat, CV_8UC1, scale * 0.05);
            cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);
        }
    }
    else if(frame->getType() == OB_FRAME_IR || frame->getType() == OB_FRAME_IR_LEFT || frame->getType() == OB_FRAME_IR_RIGHT) {
        auto videoFrame = frame->as<const ob::VideoFrame>();
        if(videoFrame->getFormat() == OB_FORMAT_Y16) {
            cv::Mat cvtMat;
            cv::Mat rawMat = cv::Mat(videoFrame->getHeight(), videoFrame->getWidth(), CV_16UC1, videoFrame->getData());
            rawMat.convertTo(cvtMat, CV_8UC1, 1.0 / 16.0f);
            cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
        }
        else if(videoFrame->getFormat() == OB_FORMAT_Y8) {
            cv::Mat rawMat = cv::Mat(videoFrame->getHeight(), videoFrame->getWidth(), CV_8UC1, videoFrame->getData());
            cv::cvtColor(rawMat * 2, rstMat, cv::COLOR_GRAY2RGB);
        }
        else if(videoFrame->getFormat() == OB_FORMAT_MJPG) {
            cv::Mat rawMat(1, videoFrame->getDataSize(), CV_8UC1, videoFrame->getData());
            rstMat = cv::imdecode(rawMat, 1);
            cv::cvtColor(rstMat * 2, rstMat, cv::COLOR_GRAY2RGB);
        }
    }
    else if(frame->getType() == OB_FRAME_ACCEL) {
        rstMat                 = cv::Mat::zeros(640, 360, CV_8UC3);
        auto        accelFrame = frame->as<ob::AccelFrame>();
        auto        value      = accelFrame->getValue();
        std::string str        = "Accel:";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 60), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" timestamp=") + std::to_string(accelFrame->getTimeStampUs()) + "us";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 120), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" x=") + std::to_string(value.x) + "m/s^2";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 180), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" y=") + std::to_string(value.y) + "m/s^2";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 240), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" z=") + std::to_string(value.z) + "m/s^2";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 300), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    else if(frame->getType() == OB_FRAME_GYRO) {
        rstMat                = cv::Mat::zeros(640, 360, CV_8UC3);
        auto        gyroFrame = frame->as<ob::GyroFrame>();
        auto        value     = gyroFrame->getValue();
        std::string str       = "Gyro:";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 60), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" timestamp=") + std::to_string(gyroFrame->getTimeStampUs()) + "us";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 120), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" x=") + std::to_string(value.x) + "rad/s";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 180), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" y=") + std::to_string(value.y) + "rad/s";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 240), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        str = std::string(" z=") + std::to_string(value.z) + "rad/s";
        cv::putText(rstMat, str.c_str(), cv::Point(8, 300), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    return rstMat;
}

// add frame info to mat
void CVWindow::drawInfo(cv::Mat &imageMat, std::shared_ptr<const ob::VideoFrame> &frame) {
    int      baseline = 0;  // Used to calculate text size and baseline
    cv::Size textSize;      // Size of the text to be drawn
    int      padding = 5;   // Padding around the text for the background

    // Helper lambda function to draw text with background
    auto putTextWithBackground = [&](const std::string &text, cv::Point origin) {
        // Getting text size for background
        textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);

        // Drawing the white background
        cv::rectangle(imageMat, origin + cv::Point(0, baseline), origin + cv::Point(textSize.width, -textSize.height) - cv::Point(0, padding),
                      cv::Scalar(255, 255, 255), cv::FILLED);

        // Putting black text on the white background
        cv::putText(imageMat, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    };

    // Drawing text with background based on frame type
    if(frame->getType() == OB_FRAME_COLOR && frame->getFormat() == OB_FORMAT_NV21) {
        putTextWithBackground("Color-NV21", cv::Point(8, 16));
    }
    else if(frame->getType() == OB_FRAME_COLOR && frame->getFormat() == OB_FORMAT_MJPG) {
        putTextWithBackground("Color-MJPG", cv::Point(8, 16));
    }
    else if(frame->getType() == OB_FRAME_DEPTH) {
        putTextWithBackground("Depth", cv::Point(8, 16));
    }
    else if(frame->getType() == OB_FRAME_IR) {
        putTextWithBackground("IR", cv::Point(8, 16));
    }
    else if(frame->getType() == OB_FRAME_IR_LEFT) {
        putTextWithBackground("LeftIR", cv::Point(8, 16));
    }
    else if(frame->getType() == OB_FRAME_IR_RIGHT) {
        putTextWithBackground("RightIR", cv::Point(8, 16));
    }

    // Timestamp information with background
    putTextWithBackground("frame timestamp(ms):  " + std::to_string(frame->getTimeStampUs()), cv::Point(8, 40));
    putTextWithBackground("system timestamp(ms): " + std::to_string(frame->getSystemTimeStampUs()), cv::Point(8, 64));
}

}  // namespace ob_smpl