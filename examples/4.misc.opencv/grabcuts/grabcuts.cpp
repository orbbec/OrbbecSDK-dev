// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"
#include "../common/cvhelper.hpp"

#include <opencv2/opencv.hpp>

int main(void) try {
    // Configure which streams to enable or disable for the Pipeline by creating a Config
    auto config = std::make_shared<ob::Config>();

    // enable depth stream with specified format
    config->enableVideoStream(OB_STREAM_DEPTH, 848., 480, 15, OB_FORMAT_Y16);
    config->enableVideoStream(OB_STREAM_COLOR, 848, 480, 15, OB_FORMAT_RGB);
    // set the frame aggregate output mode to ensure all types of frames are included in the output frameset
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    // Create a Pipeline to start stream
    auto pipeline = std::make_shared<ob::Pipeline>();
    pipeline->enableFrameSync();

    // Create a filter to align depth frame to color frame
    auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);

    // using StructuringElement for erode / dilate operations
    auto gen_element = [](int erosion_size) {
        return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erosion_size + 1, erosion_size + 1), cv::Point(erosion_size, erosion_size));
    };

    const int erosion_size = 3;
    auto      erode_less   = gen_element(erosion_size);
    auto      erode_more   = gen_element(erosion_size * 2);

    // create erode / dilate StructuringElements
    auto create_mask_from_depth = [&](cv::Mat &depth, int thresh, cv::ThresholdTypes type) {
        cv::threshold(depth, depth, thresh, 255, type);
        cv::dilate(depth, depth, erode_less);
        cv::erode(depth, depth, erode_more);
    };

    pipeline->start(config);

    for(int i = 0; i < 10; ++i) {
        pipeline->waitForFrameset();
    }

    while(true) {
        auto frameset = pipeline->waitForFrameset();
        auto alignSet = alignFilter->process(frameset);

        // Get the depth frame, color frame, and convert color Mat
        auto depthFrame = alignSet->as<ob::FrameSet>()->getFrame(OB_FRAME_DEPTH)->as<ob::VideoFrame>();
        auto colorFrame = alignSet->as<ob::FrameSet>()->getFrame(OB_FRAME_COLOR)->as<ob::VideoFrame>();
        auto colorMat = CvHelper::frameToMat(colorFrame);

        auto rawVisualize = CvHelper::colorizeDepth(depthFrame);
        // Generate "near" mask images
        cv::Mat near;
        cv::Mat far;

        cvtColor(depthMat, near, cv::COLOR_BGR2GRAY);
        create_mask_from_depth(near, 100, cv::THRESH_BINARY_INV);

        // Generate "far" mask images
        cvtColor(depthMat, far, cv::COLOR_BGR2GRAY);
        // far.setTo(255, far == 0); // Set pixels outside the "far" region to white
        create_mask_from_depth(far, 100, cv::THRESH_BINARY);

        // cv::imshow("Depth", rawVisualize);
        cv::imshow("Near", near);
        cv::imshow("Far", far);

        // GrabCut algorithm needs a mask with every pixel marked as either:
        // BGD, FGB, PR_BGD, PR_FGB
        // cv::Mat mask;
        // mask.create(near.size(), CV_8UC1);
        // mask.setTo(cv::Scalar::all(cv::GC_BGD));
        // mask.setTo(cv::GC_PR_BGD, far == 0); // Relax this to "probably background" for pixels outside "far" region
        // mask.setTo(cv::GC_FGD, near == 255); // Set pixels within the "near" region to "foreground"

        // // Apply grabcut algorithm to mask
        // cv::Mat bgdModel, fgdModel;
        // cv::grabCut(colorMat, mask, cv::Rect(), bgdModel, fgdModel, 1, cv::GC_INIT_WITH_MASK);

        // // Extract foreground and background regions from mask
        // cv::Mat3b foreground = cv::Mat3b::zeros(colorMat.rows, colorMat.cols);
        // colorMat.copyTo(foreground, (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD));

        // cv::imshow("Grabcuts", foreground);
        int key = cv::waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q') {
            break;
        }
    }

    pipeline->stop();

    exit(EXIT_SUCCESS);
}
catch(const ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    std::cout << "\nPress any key to exit.";
    ob_smpl::waitForKeyPressed();
    exit(EXIT_FAILURE);
}