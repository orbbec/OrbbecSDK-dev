#include <libobsensor/ObSensor.hpp>

#include "utils_opencv.hpp"

int main(void) try {

    // Create a pipeline.
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Get device from pipeline.
    auto device = pipe.getDevice();

    // Get sensorList from device.
    auto sensorList = device->getSensorList();

    for(uint32_t index = 0; index < sensorList->getCount(); index++) {
        // Query all supported infrared sensor type and enable the infrared stream.
        // For dual infrared device, enable the left and right infrared streams.
        // For single infrared device, enable the infrared stream.
        OBSensorType sensorType = sensorList->getSensorType(index);
        if(sensorType == OB_SENSOR_IR || sensorType == OB_SENSOR_IR_LEFT || sensorType == OB_SENSOR_IR_RIGHT) {
            // Enable the stream with specified requirements.
            config->enableVideoStream(OB_STREAM_COLOR);
        }
    }

    // Start the pipeline with default config, more info please refer to the `misc/config/OrbbecSDKConfig_v1.0.xml`.
    pipe.start();

    // Create a window for rendering, and set the size of the window.
    Window app("quick start", 1280, 720, RENDER_ONE_ROW);

    while(app) {
        // Wait for frameSet from the pipeline.
        auto frameSet = pipe.waitForFrameset();

        if(frameSet == nullptr) {
            continue;
        }

        // Get the depth raw from the frameSet.
        auto depthFrameRaw  = frameSet->getFrame(OB_FRAME_DEPTH);
        // Get the color raw from the frameSet.
        auto colorFrameRaw  = frameSet->getFrame(OB_FRAME_COLOR);

        if(depthFrameRaw == nullptr || colorFrameRaw == nullptr){
            continue;
        }

        // Get the depth frame from the depth raw.
        auto depthFrame = depthFrameRaw->as<ob::DepthFrame>();
        // Get the color frame from the color raw.
        auto colorFrame = colorFrameRaw->as<ob::ColorFrame>();

        if(depthFrame == nullptr || colorFrame == nullptr){
            continue;
        }

        // Rendering display
        app.renderFrameData({colorFrame, depthFrame});
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
