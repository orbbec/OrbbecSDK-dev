#include <openobsdk/ObSensor.hpp>

#include "window.hpp"

int main(void) try {

    // Create a pipeline with default device.
    ob::Pipeline pipe;
    // By creating config to configure which streams to enable or disable for the pipeline, here the depth stream will be enabled.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    // Enable depth stream.
    config->enableVideoStream(OB_STREAM_IR_LEFT,640,400,25,OB_FORMAT_Y12);

    auto device = pipe.getDevice();
    device->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, false);

    // Start the pipeline with config.
    pipe.start(config);
    // Create a window for rendering, and set the resolution of the window.
    Window app("DepthViewer", 1280, 720);

    while(app) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }
        // Get the depth frame from frameset.
        auto depthFrame = frameSet->getFrame(OB_FRAME_IR_LEFT)->as<ob::IRFrame>();

        // for Y16 format depth frame, print the distance of the center pixel every 30 frames.
        if(depthFrame->getIndex() % 30 == 0 && depthFrame->getFormat() == OB_FORMAT_Y16) {
            uint32_t  width  = depthFrame->getWidth();
            uint32_t  height = depthFrame->getHeight();
            std::cout<<"width: " << width << " height: " << height << std::endl;
            // float     scale  = depthFrame->getValueScale();
            // uint16_t *data   = (uint16_t *)depthFrame->getData();

            // pixel value multiplied by scale is the actual distance value in millimeters.
            // float centerDistance = data[width * height / 2 + width / 2] * scale;

            // attention: if the distance is 0, it means that the depth camera cannot detect the object (may be out of detection range).
            // std::cout << "Facing an object " << centerDistance << " mm away. " << std::endl;
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