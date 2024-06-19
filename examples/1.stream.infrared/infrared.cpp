#include <openobsdk/ObSensor.hpp>

#include "window.hpp"

int main(void) try {
    // Create a pipeline with default device.
    ob::Pipeline pipe;
    // Get the device from pipeline.
    std::shared_ptr<ob::Device> device = pipe.getDevice();
    // Get the sensor list from device.
    std::shared_ptr<ob::SensorList> sensorList = device->getSensorList();
    // Create a config for pipeline.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Distinguish between single IR and double IR.
    bool isDoubleIR = true;
    for(uint32_t index = 0; index < sensorList->count(); index++) {
        // Get the sensor type.
        auto sensorType = sensorList->type(index);
        if(sensorType == OB_SENSOR_IR) {
            isDoubleIR = false;
        }
    }

    if(!isDoubleIR){
        // Enable the infrared stream.
        config->enableVideoStream(OB_STREAM_IR, OB_WIDTH_ANY, OB_HEIGHT_ANY, 30, OB_FORMAT_Y8);
        // Start the pipeline with config
        pipe.start(config);

        // Create a window for rendering and set the resolution of the window
        Window app("InfraredViewer", 1280, 720);
        while(app) {
            // Wait for up to 100ms for a frameset in blocking mode.
            auto frameSet = pipe.waitForFrames(100);
            if(frameSet == nullptr) {
                continue;
            }

            // Render a set of frame in the window, only the infrared frame is rendered here.
            // If the open stream type is not OB_SENSOR_IR, use the getFrame interface to get the frame.
            app.addToRender(frameSet->getFrame(OB_FRAME_IR)->as<ob::IRFrame>());
        }
    }else{
        // Enable the left and right infrared streams.
        config->enableVideoStream(OB_STREAM_IR_LEFT, OB_WIDTH_ANY, OB_HEIGHT_ANY, 30, OB_FORMAT_Y8);
        config->enableVideoStream(OB_STREAM_IR_RIGHT, OB_WIDTH_ANY, OB_HEIGHT_ANY, 30, OB_FORMAT_Y8);

        // Start the pipeline with config
        pipe.start(config);

        // Create a window for rendering and set the resolution of the window
        Window app("IR_left_and_right", 1280, 360, RENDER_ONE_ROW);
        while(app) {
            // Wait for up to 100ms for a frameset in blocking mode.
            auto frameSet = pipe.waitForFrames(100);
            if(frameSet == nullptr) {
                continue;
            }

            // Render a set of frame in the window, only the infrared frame is rendered here.
            // If the open stream type is not OB_SENSOR_IR, use the getFrame interface to get the frame.
            auto leftFrame  = frameSet->getFrame(OB_FRAME_IR_LEFT);
            auto rightFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);

            if(leftFrame == nullptr || rightFrame == nullptr) {
                std::cout << "left ir frame or right ir frame is null. left frame: " << leftFrame << ", rightFrame: " << rightFrame << std::endl;
                continue;
            }

            // Render a set of frame in the window, only the infrared frame is rendered here, but it must also be passed in as an array.
         app.addToRender({ leftFrame, rightFrame });
           std::cout << "...." << std::endl;
        }
    }

    // Stop the pipeline, no frame data will be generated
    pipe.stop();
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunctionName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
