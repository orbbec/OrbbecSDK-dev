#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"

#include <chrono>
#include <thread>
#include <iomanip>

std::thread         inputWatchThread;
void                inputWatcher();
OBFrameMetadataType metadataType;
auto                metadataCount = OBFrameMetadataType::OB_FRAME_METADATA_TYPE_COUNT;

std::map<uint32_t, std::string> metadataTypeMap = {
    { 0, "OB_FRAME_METADATA_TYPE_TIMESTAMP" },
    { 1, "OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP" },
    { 2, "OB_FRAME_METADATA_TYPE_FRAME_NUMBER" },
    { 3, "OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE" },
    { 4, "OB_FRAME_METADATA_TYPE_EXPOSURE" },
    { 5, "OB_FRAME_METADATA_TYPE_GAIN" },
    { 6, "OB_FRAME_METADATA_TYPE_AUTO_WHITE_BALANCE" },
    { 7, "OB_FRAME_METADATA_TYPE_WHITE_BALANCE" },
    { 8, "OB_FRAME_METADATA_TYPE_BRIGHTNESS" },
    { 9, "OB_FRAME_METADATA_TYPE_CONTRAST" },
    { 10, "OB_FRAME_METADATA_TYPE_SATURATION" },
    { 11, "OB_FRAME_METADATA_TYPE_SHARPNESS" },
    { 12, "OB_FRAME_METADATA_TYPE_BACKLIGHT_COMPENSATION" },
    { 13, "OB_FRAME_METADATA_TYPE_HUE" },
    { 14, "OB_FRAME_METADATA_TYPE_GAMMA" },
    { 15, "OB_FRAME_METADATA_TYPE_POWER_LINE_FREQUENCY" },
    { 16, "OB_FRAME_METADATA_TYPE_LOW_LIGHT_COMPENSATION" },
    { 17, "OB_FRAME_METADATA_TYPE_MANUAL_WHITE_BALANCE" },
    { 18, "OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE" },
    { 19, "OB_FRAME_METADATA_TYPE_FRAME_RATE" },
    { 20, "OB_FRAME_METADATA_TYPE_AE_ROI_LEFT" },
    { 21, "OB_FRAME_METADATA_TYPE_AE_ROI_TOP" },
    { 22, "OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT" },
    { 23, "OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM" },
    { 24, "OB_FRAME_METADATA_TYPE_EXPOSURE_PRIORITY" },
    { 25, "OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_NAME" },
    { 26, "OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_SIZE" },
    { 27, "OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX" },
    { 28, "OB_FRAME_METADATA_TYPE_LASER_POWER" },
    { 29, "OB_FRAME_METADATA_TYPE_LASER_POWER_LEVEL" },
    { 30, "OB_FRAME_METADATA_TYPE_LASER_STATUS" },
    { 31, "OB_FRAME_METADATA_TYPE_GPIO_INPUT_DATA" },
};
int main() try {
    // Create a pipeline.
    ob::Pipeline pipe;

    // Start the pipeline with default config.
    // Modify the default configuration by the configuration file: "OrbbecSDKConfig.xml"
    pipe.start();

    // get key input
    inputWatchThread = std::thread(inputWatcher);
    inputWatchThread.detach();

    while(true) {
        // Wait for frameSet from the pipeline, the default timeout is 1000ms.
        auto frameSet = pipe.waitForFrameset();

        // get the count of frames in the frameSet
        auto frameCount = frameSet->getCount();

        for(uint32_t i = 0; i < frameCount; i++) {
            // get the frame from frameSet
            auto frame      = frameSet->getFrame(i);
            auto frameIndex = frame->index();
            // get the metadata of the frame, and print the metadata every 30 frames
            if(frameIndex % 30 == 0) {
                std::cout << std::endl;
                for(uint32_t j = 0; j < static_cast<uint32_t>(metadataCount); j++) {
                    // if the frame has the metadata, get the metadata value
                    if(frame->hasMetadata(static_cast<OBFrameMetadataType>(j))) {
                        std::cout << "metadata type: " << std::left << std::setw(50) << metadataTypeMap[j]
                                  << " metadata value: " << frame->getMetadataValue(static_cast<OBFrameMetadataType>(j)) << std::endl;
                    }
                }
            }
        }
    }
    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    std::cout << "\nPress any key to exit.";
    ob_smpl::waitForKeyPressed();
    exit(EXIT_FAILURE);
}

void inputWatcher() {
    char input = ob_smpl::waitForKeyPressed();
    if(input == ESC_KEY) {
        exit(EXIT_SUCCESS);
    }
}