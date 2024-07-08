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
        if(sensorType == OB_SENSOR_IR || sensorType == OB_SENSOR_IR_LEFT || sensorType == OB_SENSOR_IR_RIGHT || sensorType == OB_SENSOR_COLOR
           || sensorType == OB_SENSOR_DEPTH) {
            // Enable the stream with specified requirements.
            config->enableVideoStream(ob::TypeHelper::convertSensorTypeToStreamType(sensorType));
        }
    }

    std::mutex                    frameMutex;
    std::shared_ptr<ob::FrameSet> renderframeSet = nullptr;

    // Start the pipeline with callback.
    pipe.start(config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lock(frameMutex);
        renderframeSet = frameSet;
    });

    // Create a window for rendering, and set the size of the window.
    ob_smpl::CVWindow win("quick start", 1280, 720, ob_smpl::RENDER_GRID);

    while(win.run()) {
        std::lock_guard<std::mutex> lock(frameMutex);

        if(renderframeSet == nullptr) {
            continue;
        }

        // Rendering display
        win.pushFramesToShow(renderframeSet);
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
