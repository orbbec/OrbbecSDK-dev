#include "window.hpp"

#include <openobsdk/ObSensor.h>

int main() try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    auto device     = pipe.getDevice();
    auto sensor     = device->getSensor(OB_SENSOR_DEPTH);
    auto filterList = sensor->getRecommendedFilters();

    for(auto &filter: filterList) {
        std::cout << "\t - " << filter->getName() << ": " << filter->isEnabled() << std::endl;
    }

    // By creating config to configure which streams to enable or disable for the pipeline, here the depth stream will be enabled
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(OB_STREAM_DEPTH);

    // Start the pipeline with config
    pipe.start(config);

    // Create a window for rendering, and set the resolution of the window
    Window app("PostProcessing", 640, 480);

    while(app) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
        if(!depthFrame) {
            continue;
        }

        for(auto &filter: filterList) {
            depthFrame = filter->process(depthFrame);
        }

        // Render frame in the window
        app.addToRender(depthFrame);
    }

    // Stop the pipeline
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunctionName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
