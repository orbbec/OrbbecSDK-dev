// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"
#include "utils_opencv.hpp"

#include <memory>

// Helper function to convert depth frame to colorized image
cv::Mat colorizeDepth(std::shared_ptr<ob::frame> depthFrame) {
    cv::Mat rstMat;
    cv::Mat cvtMat;

    const uint32_t width  = depthFrame->as<ob::VideoFrame>()->getWidth();
    const uint32_t height = depthFrame->as<ob::VideoFrame>()->getHeight();
    // depth frame pixel value multiply scale to get distance in millimeter
    const float scale = depthFrame->as<ob::DepthFrame>()->getValueScale();

    // Get raw depth data from depth frame
    cv::Mat rawMat = cv::Mat(depthFrame->getHeight(), depthFrame->getWidth(), CV_16UC1, depthFrame->getData());

    // normalization to 0-255. 0.032f is 256/8000, to limit the range of depth to 8000mm
    rawMat.convertTo(cvtMat, CV_32F, scale * 0.032f);

    // apply gamma correction to enhance the contrast for near objects
    cv::pow(cvtMat, 0.6f, cvtMat);

    //  convert to 8-bit
    cvtMat.convertTo(cvtMat, CV_8UC1, 10);  // multiplier 10 is to normalize to 0-255 (nearly) after applying gamma correction

    // apply colormap
    cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);

    return rstMat;
}

int main(void) try {
    // Configure which streams to enable or disable for the Pipeline by creating a Config
    auto config = std::make_shared<ob::Config>();

    // enable depth stream with specified format
    config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);

    std::shared_ptr<ob::Pipeline> pipeline = std::make_shared<ob::Pipeline>();
    pipeline->start(config);

    while(true) {
        auto frames     = pipeline->waitForFrames();
        auto depthFrame = frames->getFrame(OB_FRAME_DEPTH);

        // Color mapping the depth data in order to visualize it intuitively
        cv::Mat colorMat = colorizeDepth(depthFrame);

        cv::imshow("Display Depth Image", colorMat);
        int key = cv::waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q') {
            break;
        }
    }

    pipeline->stop();

    exit(EXIT_SUCCESS);
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    std::cout << "\nPress any key to exit.";
    ob_smpl::waitForKeyPressed();
    exit(EXIT_FAILURE);
}