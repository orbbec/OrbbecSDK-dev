// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"
#include "utils_opencv.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>

bool getFirmwarePath(std::string &firmwarePath);
void onDeviceConnected(const std::string &prompt, std::shared_ptr<ob::DeviceList> deviceList);
void getDevice(const std::shared_ptr<ob::Context> &context);

bool firstCall     = true;
bool firstMainCall = true;
bool rebootDone    = false;

std::condition_variable     cond;
std::mutex                  mutex;
std::shared_ptr<ob::Device> device = nullptr;

// Callback function for device firmware update progress
ob::Device::DeviceFwUpdateCallback firmwareUpdateCallback = [&](OBFwUpdateState state, const char *message, uint8_t percent) {
    if(!firstCall) {
        // Move cursor up 3 lines
        std::cout << "\033[3F";  // ANSI code to move cursor up 3 lines
    }
    else {
        firstCall = false;
    }

    // Clear and print the progress line
    std::cout << "\033[K";  // Clear the current line
    std::cout << "Progress: " << static_cast<uint32_t>(percent) << "%" << std::endl;

    // Clear and print the status line
    std::cout << "\033[K";  // Clear the current line
    std::cout << "Status: ";
    switch(state) {
    case STAT_VERIFY_SUCCESS:
        std::cout << "Image file verification success";
        break;
    case STAT_FILE_TRANSFER:
        std::cout << "File transfer in progress";
        break;
    case STAT_DONE:
        std::cout << "Update completed";
        break;
    case STAT_IN_PROGRESS:
        std::cout << "Upgrade in progress";
        break;
    case STAT_START:
        std::cout << "Starting the upgrade";
        break;
    case STAT_VERIFY_IMAGE:
        std::cout << "Verifying image file";
        break;
    case ERR_VERIFY:
        std::cout << "Verification failed";
        break;
    case ERR_PROGRAM:
        std::cout << "Program execution failed";
        break;
    case ERR_ERASE:
        std::cout << "Flash parameter failed";
        break;
    case ERR_FLASH_TYPE:
        std::cout << "Flash type error";
        break;
    case ERR_IMAGE_SIZE:
        std::cout << "Image file size error";
        break;
    case ERR_OTHER:
        std::cout << "Other error";
        break;
    case ERR_DDR:
        std::cout << "DDR access error";
        break;
    case ERR_TIMEOUT:
        std::cout << "Timeout error";
        break;
    default:
        std::cout << "Unknown status";
        break;
    }
    std::cout << std::endl;

    // Clear and print the message line
    std::cout << "\033[K";  // Clear the current line
    std::cout << "Message: " << message << std::endl;
    std::cout << std::flush;
};

int main() try {
    std::shared_ptr<ob::Context> context = std::make_shared<ob::Context>();
    context->setDeviceChangedCallback([](std::shared_ptr<ob::DeviceList> removedDeviceList, std::shared_ptr<ob::DeviceList> addDeviceList) {
        onDeviceConnected("connected", addDeviceList);
        onDeviceConnected("disconnected", removedDeviceList);
    });

    getDevice(context);

    while(true) {
        firstCall    = true;
        std::string firmwarePath;

        if(!getFirmwarePath(firmwarePath)) {
            continue;
        }

        try {
            // Update the device firmware, set async false to block until the update is complete.
            device->updateFirmware(firmwarePath.c_str(), firmwareUpdateCallback, false);
        }
        catch(ob::Error &e) {
            // If the update fails, will throw an exception.
            std::cerr << "\nThe upgrade was interrupted! An error occurred! " << std::endl;
            std::cerr << "Error message: " << e.what() << std::endl;
            std::cout << "Press any key to selet another firmware file and try again." << std::endl;
            ob_smpl::waitForKeyPressed();
            continue;
        }

        // Reboot the device to apply the changes
        std::cout << "Please choose one of the following options:" << std::endl;
        std::cout << "  [R/r]           - Reboot the device to apply the firmware update\n";
        std::cout << "  [Q/q]           - Quit the program\n";
        std::cout << "  [Any other key] - Perform another firmware update\n";
        std::cout << "---------------------------------------------------------------------------\n";
        std::cout << "Your input: ";
        char input = ob_smpl::waitForKeyPressed();
        std::cout << input << std::endl;
        if(input == 'R' || input == 'r') {
            std::cout << "\nThe device is rebooting, please wait a moment..." << std::endl;
            rebootDone = false;
            device->reboot();
            getDevice(context);
        }
        else if(input == 'Q' || input == 'q') {
            break;
        }
        else {
            continue;
        }
    }
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    std::cout << "\nPress any key to exit.";
    ob_smpl::waitForKeyPressed();
    exit(EXIT_FAILURE);
}

void onDeviceConnected(const std::string &prompt, std::shared_ptr<ob::DeviceList> deviceList) {
    if(deviceList->getCount() == 0) {
        return;
    }

    if(prompt == "connected") {
        rebootDone = true;
        cond.notify_all();
    }
    else {
        std::cout << "Device disconnected: " << deviceList->getName(0) << std::endl;
    }
}

bool getFirmwarePath(std::string &firmwarePath) {
    std::cout << "\nEnter the path to the firmware file (.bin) or press 'Q' to quit: ";
    std::getline(std::cin, firmwarePath);

    if(firmwarePath == "Q" || firmwarePath == "q") {
        exit(EXIT_SUCCESS);
    }

    if(firmwarePath.size() < 4 || firmwarePath.substr(firmwarePath.size() - 4) != ".bin") {
        std::cerr << "Error: Invalid firmware file extension. Please provide a path to a .bin file." << std::endl;
        std::cerr << "Example: /path/to/firmware.bin" << std::endl << std::endl;
        return false;
    }

    std::ifstream firmwareFile(firmwarePath);
    if(!firmwareFile) {
        std::cerr << "Error: Firmware file not found at the specified path: " << firmwarePath << std::endl;
        std::cerr << "Please ensure the file exists and provide the correct path." << std::endl;
        std::cerr << "Example: /path/to/firmware.bin" << std::endl << std::endl;
        return false;
    }
    firmwareFile.close();

    std::cout << "Firmware file confirmed: " << firmwarePath << std::endl;
    std::cout << "Starting firmware update, please wait..." << std::endl << std::endl;
    return true;
}

void getDevice(const std::shared_ptr<ob::Context> &context) {
    if(firstMainCall) {
        firstMainCall                              = false;
        std::shared_ptr<ob::DeviceList> deviceList = context->queryDeviceList();
        if(deviceList->getCount() == 0) {
            std::cout << "No device found. Please connect a device first." << std::endl;
            std::cout << "Waiting for device connection..." << std::endl;
            {
                std::unique_lock<std::mutex> mtx(mutex);
                cond.wait(mtx, [&]() { return rebootDone; });
            }
        }
        else {
            device = deviceList->getDevice(0);
            std::cout << "Device found: " << device->getDeviceInfo()->getName() << std::endl;
            std::cout << "Current device firmware version: " << device->getDeviceInfo()->getFirmwareVersion() << std::endl;
        }
    }
    else {
        {
            std::unique_lock<std::mutex> mtx(mutex);
            cond.wait(mtx, [&]() { return rebootDone; });
        }
        std::shared_ptr<ob::DeviceList> deviceList = context->queryDeviceList();
        device                                     = deviceList->getDevice(0);
        std::shared_ptr<ob::DeviceInfo> deviceInfo = device->getDeviceInfo();
        std::cout << "\033[2J\033[H";  // Clear the screen
        std::cout << "Device reboot completed, Device name: " << deviceInfo->getName() << std::endl;
        std::cout << "Current device firmware version: " << deviceInfo->getFirmwareVersion() << std::endl;
    }
}