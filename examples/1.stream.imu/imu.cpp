#include <libobsensor/ObSensor.hpp>

#include <mutex>
#include <iostream>

#include "utils.hpp"

void printImuValue(OBFloat3D obFloat3d, uint64_t index, uint64_t timeStampUs, float temperature, OBFrameType type) {
    std::cout << index << std::endl;
    std::cout << type << " Frame: \n\r{\n\r"
              << "  tsp = " << timeStampUs << "\n\r"
              << "  temperature = " << temperature << "\n\r"
              << "  " << type << ".x = " << obFloat3d.x << " m/s^2"
              << "\n\r"
              << "  " << type << ".y = " << obFloat3d.y << " m/s^2"
              << "\n\r"
              << "  " << type << ".z = " << obFloat3d.z << " m/s^2"
              << "\n\r"
              << "}\n\r" << std::endl;
}
int main() try {

    // Create a pipeline with default device.
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Enable Accel stream.
    config->enableAccelStream();

    // Enable Gyro stream.
    config->enableGyroStream();

    // Only FrameSet that contains all types of data frames will be output.
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    // Start the pipeline with config.
    pipe.start(config);

    while(true) {
        if(_kbhit() && _getch() == ESC) {
            break;
        }

        auto frameSet = pipe.waitForFrameset();

        auto accelFrameRaw    = frameSet->getFrame(OB_FRAME_ACCEL);
        auto accelFrame       = accelFrameRaw->as<ob::AccelFrame>();
        auto accelIndex       = accelFrame->getIndex();
        auto accelTimeStampUs = accelFrame->getTimeStampUs();
        auto accelTemperature = accelFrame->getTemperature();
        auto accelType        = accelFrame->getType();
        if(accelIndex % 50 == 0) {  // print information every 50 frames.
            auto accelValue = accelFrame->getValue();
            printImuValue(accelValue, accelIndex, accelTimeStampUs, accelTemperature, accelType);
        }

        auto gyroFrameRaw    = frameSet->getFrame(OB_FRAME_GYRO);
        auto gyroFrame       = gyroFrameRaw->as<ob::GyroFrame>();
        auto gyroIndex       = gyroFrame->getIndex();
        auto gyrotimeStampUs = gyroFrame->getTimeStampUs();
        auto gyroTemperature = gyroFrame->getTemperature();
        auto gyroType        = gyroFrame->getType();
        if(gyroIndex % 50 == 0) {  // print information every 50 frames.
            auto gyroValue = gyroFrame->getValue();
            printImuValue(gyroValue, gyroIndex, gyrotimeStampUs, gyroTemperature, gyroType);
        }
    }

    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}