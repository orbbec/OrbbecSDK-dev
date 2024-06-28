#include <libobsensor/ObSensor.hpp>

#include "utils_opencv.hpp"

#include <iostream>


const char* sensorTypes[] =  {
    "OB_SENSOR_UNKNOWN",
    "OB_SENSOR_IR",
    "OB_SENSOR_COLOR",
    "OB_SENSOR_DEPTH",
    "OB_SENSOR_ACCEL",
    "OB_SENSOR_GYRO",
    "OB_SENSOR_IR_LEFT",
    "OB_SENSOR_IR_RIGHT",
    "OB_SENSOR_RAW_PHASE",
    "OB_SENSOR_COUNT"
};

const char *streamTypes[] = { "OB_FORMAT_YUYV",  "OB_FORMAT_YUY2",       "OB_FORMAT_UYVY", "OB_FORMAT_NV12",   "OB_FORMAT_NV21",   "OB_FORMAT_MJPG",
                              "OB_FORMAT_H264",  "OB_FORMAT_H265",       "OB_FORMAT_Y16",  "OB_FORMAT_Y8",     "OB_FORMAT_Y10",    "OB_FORMAT_Y11",
                              "OB_FORMAT_Y12",   "OB_FORMAT_GRAY",       "OB_FORMAT_HEVC", "OB_FORMAT_I420",   "OB_FORMAT_ACCEL",  "OB_FORMAT_GYRO",
                              "OB_FORMAT_POINT", "OB_FORMAT_RGB_POINT",  "OB_FORMAT_RLE",  "OB_FORMAT_RGB",    "OB_FORMAT_BGR",    "OB_FORMAT_Y14",
                              "OB_FORMAT_BGRA",  "OB_FORMAT_COMPRESSED", "OB_FORMAT_RVL",  "OB_FORMAT_Z16",    "OB_FORMAT_YV12",   "OB_FORMAT_BA81",
                              "OB_FORMAT_RGBA",  "OB_FORMAT_BYR2",       "OB_FORMAT_RW16", "OB_FORMAT_DISP16", "OB_FORMAT_UNKNOWN" };

const char *rateTypes[] = {
    "OB_SAMPLE_RATE_UNKNOWN",
    "OB_SAMPLE_RATE_1_5625_HZ",
    "OB_SAMPLE_RATE_3_125_HZ",
    "OB_SAMPLE_RATE_6_25_HZ",
    "OB_SAMPLE_RATE_12_5_HZ",
    "OB_SAMPLE_RATE_25_HZ",
    "OB_SAMPLE_RATE_50_HZ",
    "OB_SAMPLE_RATE_100_HZ",
    "OB_SAMPLE_RATE_200_HZ",
    "OB_SAMPLE_RATE_500_HZ",
    "OB_SAMPLE_RATE_1_KHZ",
    "OB_SAMPLE_RATE_2_KHZ",
    "OB_SAMPLE_RATE_4_KHZ",
    "OB_SAMPLE_RATE_8_KHZ",
    "OB_SAMPLE_RATE_16_KHZ",
    "OB_SAMPLE_RATE_32_KHZ",
};

// enumerate stream profiles
void enumerateStreamProfiles(std::shared_ptr<ob::Sensor> sensor){
    // Get the list of stream profiles.
    auto streamProfileList = sensor->getStreamProfileList();
    // Get the sensor type.
    auto sensorType = sensor->getType();
    for(uint32_t index = 0; index < streamProfileList->getCount(); index++) {
        // Get the stream profile.
        auto profile = streamProfileList->getProfile(index);
        if(sensorType == OB_SENSOR_IR || sensorType == OB_SENSOR_COLOR || sensorType == OB_SENSOR_DEPTH || sensorType == OB_SENSOR_IR_LEFT || sensorType == OB_SENSOR_IR_RIGHT){
            // Get the video profile.
            auto videoProfile = profile->as<ob::VideoStreamProfile>();
            std::cout << index << "." << "format: " << streamTypes[profile->getFormat()] << " " << "revolution: " << videoProfile->getWidth() << "*"
                      << videoProfile->getHeight() << " " << "fps: " << videoProfile->getFps() << std::endl;
        }else if(sensorType == OB_SENSOR_ACCEL){
            //Get the acc profile.
            auto accProfile = profile->as<ob::AccelStreamProfile>();
            std::cout << "acc rate: " << rateTypes[accProfile->getSampleRate()] << std::endl;
        }else if(sensorType == OB_SENSOR_GYRO){
            //Get the gyro profile.
            auto gyroProfile = profile->as<ob::GyroStreamProfile>();
            std::cout << "gyro rate: " << rateTypes[gyroProfile->getSampleRate()] << std::endl;
        }else{
            break;
        }
    }
}

// enumerate sensors
void enumerateSensors(std::shared_ptr<ob::Device> device){
    while(true){
        // Get the list of sensors.
        auto sensorList = device->getSensorList();
        for(uint32_t index = 0; index < sensorList->getCount(); index++) {
            // Get the sensor type.
            auto sensorType = sensorList->getSensorType(index);
            std::cout << index << "." <<"sensor type:" << sensorTypes[sensorType] << std::endl;
        }

        std::cout << "Select a sensor to enumerate its streams(input sensor index or \'q\' to enumerate device): " << std::endl;

        // Select a sensor
        std::string sensorSelected;
        std::cin >> sensorSelected;
        if(sensorSelected == "q" || sensorSelected == "Q") {
            break;
        }
        enumerateStreamProfiles(sensorList->getSensor(std::stoul(sensorSelected)));
    }
}

int main(void) try {

    // Create a Context.
    ob::Context context;

    while(true){
        // Query the list of connected devices
        auto deviceList = context.queryDeviceList();

        if(deviceList->getCount() < 1) {
            std::cout << "Not found device !" << std::endl;
            return -1;
        }

        std::cout << "enumerated devices: " << std::endl;
        for(uint32_t index = 0; index < deviceList->getCount(); index++) {
            auto deviceInfo = deviceList->getDevice(index)->getDeviceInfo();
            std::cout << " - " << index << ". name: " << deviceInfo->getName() << " pid: " << deviceInfo->getPid() << " SN: " << deviceInfo->getSerialNumber()
                      << std::endl;
        }

        std::cout << "enumerate sensors of device (input device index or \'q\' to exit program):" << std::endl;

        // select a device
        std::string device_selected;
        std::cin >> device_selected;
        if(device_selected == "q" || device_selected == "Q"){
            break;
        }

        //  Get the device
        auto device = deviceList->getDevice(std::stoul(device_selected));
        enumerateSensors(device);
    }

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
