#include <openobsdk/ObSensor.hpp>

#include "window.hpp"

// #include "utils.hpp"

int main(void) try {

    // Create a pipeline.
    ob::Pipeline pipe;

    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`.
    pipe.start();

    // Create a window for rendering, and set the size of the window.
    Window app("MultiStream", 1280, 720, RENDER_ONE_ROW);

    while(app) {
        // Wait for frameSet from the pipeline.
        auto frameSet = pipe.waitForFrames();
        if(frameSet == nullptr) {
            continue;
        }

        // Get the depth from the frameSet.
        auto dFrame = frameSet->getFrame(OB_FRAME_DEPTH);
        // Get the color from the frameSet.
        auto cFrame = frameSet->getFrame(OB_FRAME_COLOR);
        if(dFrame && cFrame) {
            auto depthFrame = dFrame->as<ob::DepthFrame>();
            auto colorFrame = cFrame->as<ob::ColorFrame>();

            if((depthFrame != nullptr) && (colorFrame != nullptr)){
                // Render frame in the window.
                app.addToRender({colorFrame, depthFrame});
            }
        }

    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunctionName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
