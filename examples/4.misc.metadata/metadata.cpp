#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"

#include <chrono>
#include <thread>

std::thread         inputWatchThread;
void                inputWatcher();
OBFrameMetadataType metadataType;
auto                metadataCount = OBFrameMetadataType::OB_FRAME_METADATA_TYPE_COUNT;
int                 main() try {
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
            auto frame     = frameSet->getFrame(i);
            auto frameType = frameSet->type();
            std::cout << "---------" << "frame type: " << frameType << " ---------" << std::endl;
            // get the metadata of the frame
            for(uint32_t j = 0; j < static_cast<uint32_t>(metadataCount); j++) {
                // if the frame has the metadata, get the metadata value
                if(frame->hasMetadata(static_cast<OBFrameMetadataType>(j))) {
                    std::cout << "metadata type: " << static_cast<OBFrameMetadataType>(j)
                              << " metadata value: " << frame->getMetadataValue(static_cast<OBFrameMetadataType>(j)) << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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