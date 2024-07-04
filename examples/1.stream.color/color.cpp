#include <libobsensor/ObSensor.hpp>

#include "utils_opencv.hpp"

int main(void) try {

    // Create a pipeline with default device.
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Enable color video stream.
    config->enableVideoStream(OB_STREAM_COLOR);

    // Start the pipeline with config.
    pipe.start(config);

    // Create a window for rendering and set the resolution of the window.
    Window app("ColorViewer", 1280, 720);

    while(app) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrameset();
        if(frameSet == nullptr) {
            continue;
        }

        // get color frame from frameset.
        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
        // Render colorFrame.
        app.renderFrameData(colorFrame);
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
