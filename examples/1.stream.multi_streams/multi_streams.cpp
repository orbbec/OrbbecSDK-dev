#include <libobsensor/ObSensor.h>

#include "utils_opencv.hpp"

#include <mutex>
#include <thread>

int main(void) try {

    // Create a pipeline with default device.
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Enumerate and config all sensors.
    auto device = pipe.getDevice();

    // Get sensor list from device.
    auto sensorList = device->getSensorList();

    for(uint32_t i = 0; i < sensorList->getCount(); i++) {
        // Get sensor type.
        auto sensorType = sensorList->getSensorType(i);
        if(sensorType == OB_SENSOR_GYRO || sensorType == OB_SENSOR_ACCEL) {
            continue;
        }
        // Get frame type based on sensor type.
        auto streamType = ob::TypeHelper::convertSensorTypeToStreamType(sensorType);
        // enable the stream.
        config->enableVideoStream(streamType);
    }

    // Start the pipeline with config
    std::mutex                          frameMutex;
    std::shared_ptr<const ob::FrameSet> renderframeSet;
    pipe.start(config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lock(frameMutex);
        renderframeSet = frameSet;
    });

    // The IMU frame rate is much faster than the video, so it is advisable to use a separate pipeline to obtain IMU data.
    auto                                dev         = pipe.getDevice();
    auto                                imuPipeline = std::make_shared<ob::Pipeline>(dev);
    std::mutex                          imuFrameMutex;
    std::shared_ptr<const ob::FrameSet> renderImuFrameSet;

    std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
    // enable gyro stream.
    imuConfig->enableGyroStream();
    // enable accel stream.
    imuConfig->enableAccelStream();
    // start the imu pipeline.
    imuPipeline->start(imuConfig, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lockimu(imuFrameMutex);
        renderImuFrameSet = frameSet;
    });

    // Create a window for rendering and set the resolution of the window
    Window app("MultiStream", 1280, 720, RENDER_GRID);
    while(app) {
        std::lock_guard<std::mutex> lockimu(imuFrameMutex);
        std::lock_guard<std::mutex> lock(frameMutex);

        if(renderframeSet == nullptr || renderImuFrameSet == nullptr) {
            continue;
        }
        // Render camera and imu frameset.
        app.renderFrame({ renderframeSet, renderImuFrameSet });
    }

    // Stop the Pipeline, no frame data will be generated.
    pipe.stop();

    // Stop the IMU Pipeline, no frame data will be generated.
    imuPipeline->stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
