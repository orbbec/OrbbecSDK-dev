#include <openobsdk/ObSensor.hpp>

#include "window.hpp"

// #include "utils.hpp"

int main(int argc, char **argv) try {

    // Create a pipeline.
    ob::Pipeline pipe;

    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`.
    pipe.start();

    // Create a window for rendering, and set the size of the window.
    Window app("MultiStream", 1280, 360, RENDER_ONE_ROW);

    while(app) {
        // Wait for frameSet from the pipeline.
        auto frameSet = pipe.waitForFrames();
        if(frameSet == nullptr) {
            continue;
        }

        // Get the depth from the frameSet.
        auto depthFrame = frameSet->depthFrame();
        // Get the color from the frameSet.
        auto colorFrame = frameSet->colorFrame();

        if((depthFrame != nullptr) && (colorFrame != nullptr)){
            // Render frame in the window.
            app.addToRender({colorFrame, depthFrame});
        }
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
