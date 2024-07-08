#include "utils_opencv.hpp"

#include <libobsensor/ObSensor.h>

int main() try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    // Get the device and sensor, and get the list of recommended filters for the sensor
    auto device     = pipe.getDevice();
    auto sensor     = device->getSensor(OB_SENSOR_DEPTH);
    auto filterList = sensor->getRecommendedFilters();

    // Print the list of recommended filters
    std::cout << filterList.size() << " post processing filters recommended:" << std::endl;
    for(auto &filter: filterList) {
        std::cout << "\t - " << filter->getName() << ": " << (filter->isEnabled() ? "enabled" : "disabled") << std::endl;
        auto configSchemaVec = filter->getConfigSchemaVec();
        // Print the config schema for each filter
        for(auto &configSchema: configSchemaVec) {
            std::cout << "\t\t - {" << configSchema.name << ", " << configSchema.type << ", " << configSchema.min << ", " << configSchema.max << ", "
                      << configSchema.step << ", " << configSchema.def << ", " << configSchema.desc << "}" << std::endl;
        }
        if(filter->getName() == "DecimationFilter") {
            filter->setConfigValue("decimate", 2.1);
            filter->enable(true);
        }
    }

    // Create a config with depth stream enabled
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(OB_STREAM_DEPTH);

    // Start the pipeline with config
    pipe.start(config);

    // Create a window for rendering, and set the resolution of the window
    ob_smpl::CVWindow win("PostProcessing", 1280, 720);

    while(win.run()) {
        // Wait for up to 1000ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrameset(1000);
        if(frameSet == nullptr) {
            continue;
        }

        // Get the depth frame from the frameset
        auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
        if(!depthFrame) {
            continue;
        }

        // Apply the recommended filters to the depth frame
        for(auto &filter: filterList) {
            if(filter->isEnabled()) {  // Only apply enabled filters
                depthFrame = filter->process(depthFrame);
            }
        }

        // Render frame in the window
        win.pushFramesToShow(depthFrame);
    }

    // Stop the pipeline
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
