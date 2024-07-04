#include <libobsensor/ObSensor.hpp>

#include "utils_opencv.hpp"

int main(void) try {
    // Create a pipeline.
    ob::Pipeline pipe;

    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`.
    pipe.start();

    // Create a window for rendering, and set the size of the window.
    Window app("quick start", 1280, 480, RENDER_ONE_ROW);

    while(app) {
        // Wait for frameSet from the pipeline, the default timeout is 1000ms.
        auto frameSet = pipe.waitForFrameset();

        // If timeout without getting frameSet, continue to wait for next frameSet.
        if(frameSet == nullptr) {
            continue;
        }

        // Rendering display
        app.renderFrame(frameSet);
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
