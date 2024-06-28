#include "utils_opencv.hpp"

#include <libobsensor/ObSensor.h>

#include <mutex>
#include <thread>

OBStreamType SensorTypeToStreamType(OBSensorType sensorType) {
    switch(sensorType) {
    case OB_SENSOR_COLOR:
        return OB_STREAM_COLOR;
    case OB_SENSOR_DEPTH:
        return OB_STREAM_DEPTH;
    case OB_SENSOR_IR:
        return OB_STREAM_IR;
    case OB_SENSOR_IR_LEFT:
        return OB_STREAM_IR_LEFT;
    case OB_SENSOR_IR_RIGHT:
        return OB_STREAM_IR_RIGHT;
    case OB_SENSOR_GYRO:
        return OB_STREAM_GYRO;
    case OB_SENSOR_ACCEL:
        return OB_STREAM_ACCEL;
    default:
        return OB_STREAM_UNKNOWN;
    }
}

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
        auto streamType = SensorTypeToStreamType(sensorType);
        config->enableVideoStream(streamType);
    }

    // Start the pipeline with config
    std::mutex                                        frameMutex;
    std::map<OBFrameType, std::shared_ptr<const ob::Frame>> frameMap;
    pipe.start(config, [&](std::shared_ptr<ob::FrameSet> frameset) {
        uint32_t count = frameset->getCount();
        for(uint32_t i = 0; i < count; i++) {
            auto                         frame = frameset->getFrameByIndex(i);
            std::unique_lock<std::mutex> lk(frameMutex);
            frameMap[frame->getType()] = frame;
        }
    });

    // The IMU frame rate is much faster than the video, so it is advisable to use a separate pipeline to obtain IMU data.
    auto                                              dev         = pipe.getDevice();
    auto                                              imuPipeline = std::make_shared<ob::Pipeline>(dev);
    std::mutex                                        imuFrameMutex;
    std::map<OBFrameType, std::shared_ptr<const ob::Frame>> imuFrameMap;
    try {
        std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
        imuConfig->enableGyroStream();
        imuConfig->enableAccelStream();
        imuPipeline->start(imuConfig, [&](std::shared_ptr<ob::FrameSet> frameset) {
            uint32_t count = frameset->getCount();
            for(uint32_t i = 0; i < count; i++) {
                const auto                   frame = frameset->getFrameByIndex(i);
                std::unique_lock<std::mutex> lk(imuFrameMutex);
                imuFrameMap[frame->getType()] = frame;
            }
        });
    }
    catch(...) {
        std::cout << "IMU sensor not found!" << std::endl;
        imuPipeline.reset();
    }

    // Create a window for rendering and set the resolution of the window
    Window app("MultiStream", 1280, 720, RENDER_GRID);
    while(app) {
        std::vector<std::shared_ptr<const ob::Frame>> framesForRender;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            for(auto &frame: frameMap) {
                framesForRender.push_back(frame.second);
            }
        }
        {
            std::unique_lock<std::mutex> lock(imuFrameMutex);
            for(auto &frame: imuFrameMap) {
                framesForRender.push_back(frame.second);
            }
        }
        app.addToRender(framesForRender);
    }

    // Stop the Pipeline, no frame data will be generated.
    pipe.stop();

    if(imuPipeline) {
        // Stop the imu Pipeline, no frame data will be generated.
        imuPipeline->stop();
    }
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
