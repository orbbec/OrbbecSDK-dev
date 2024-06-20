#include "window.hpp"

#include <openobsdk/ObSensor.h>

int main() try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
    auto profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);

    std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
    try {
        // Find the corresponding profile according to the specified format, first look for the y16 format
        depthProfile = profiles->getVideoStreamProfile(640, OB_HEIGHT_ANY, OB_FORMAT_Y16, 30);
    }
    catch(ob::Error &e) {
        // If the specified format is not found, search for the default profile to open the stream
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(profiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
        std::cerr << "function:" << e.getFunctionName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType()
                  << std::endl;
    }

    // By creating config to configure which streams to enable or disable for the pipeline, here the depth stream will be enabled
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(depthProfile);

    // Obtain the recommended list of post-processing depth filters
    auto obFilterList = pipe.getDevice()->getSensor(OB_SENSOR_DEPTH)->getRecommendedFilters();

    std::shared_ptr<ob::DecimationFilter> decFilter;
    for(uint32_t i = 0; i < obFilterList->count(); i++) {
        auto filter = obFilterList->getFilter(i);
        std::cout << "Depth recommended post processor filter type: " << filter->type() << std::endl;
        if(filter->is<ob::DecimationFilter>()) {
            decFilter = filter->as<ob::DecimationFilter>();
        }
    }

    // Start the pipeline with config
    pipe.start(config);

    // Create a window for rendering, and set the resolution of the window
    Window app("PostProcessing", depthProfile->width(), depthProfile->height());

    bool resizeWindow = false;
    if(decFilter && decFilter->isEnabled()) {
        resizeWindow = true;
    }
    while(app) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH)->as<ob::DepthFrame>();
        if(depthFrame) {
            for(uint32_t i = 0; i < obFilterList->count(); i++) {
                auto filter = obFilterList->getFilter(i);
                if(filter->isEnabled()) {
                    auto newFrame = filter->process(depthFrame);
                    depthFrame    = newFrame->as<ob::DepthFrame>();
                }
            }
        }

        // for Y16 format depth frame, print the distance of the center pixel every 30 frames
        if(depthFrame->getIndex() % 30 == 0 && depthFrame->getFormat() == OB_FORMAT_Y16) {
            uint32_t  width  = depthFrame->getWidth();
            uint32_t  height = depthFrame->getHeight();
            float     scale  = depthFrame->getValueScale();
            uint16_t *data   = (uint16_t *)depthFrame->getData();

            // pixel value multiplied by scale is the actual distance value in millimeters
            float centerDistance = data[width * height / 2 + width / 2] * scale;

            // attention: if the distance is 0, it means that the depth camera cannot detect the object（may be out of detection range）
            std::cout << "Facing an object " << centerDistance << " mm away. " << std::endl;
        }

        if(resizeWindow) {
            app.resize(depthFrame->getWidth(), depthFrame->getHeight());
            resizeWindow = false;
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
