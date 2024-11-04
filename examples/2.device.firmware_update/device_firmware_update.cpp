// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"
#include "utils_opencv.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>

bool getFirmwarePath(std::string &firmwarePath);
void printDeviceList();

bool firstCall = true;
std::vector<std::shared_ptr<ob::Device>> devices;

int main() try {
    // Create a context and set the device changed callback function
    std::shared_ptr<ob::Context> context = std::make_shared<ob::Context>();

    // Get connected devices from the context
    std::shared_ptr<ob::DeviceList> deviceList = context->queryDeviceList();
    if(deviceList->getCount() == 0) {
        std::cout << "No device found. Please connect a device first!" << std::endl;
        std::cout << "Press any key to exit..." << std::endl;
        ob_smpl::waitForKeyPressed();
        return 0;
    }

    for(int i = 0; i < deviceList->getCount(); ++i) {
        devices.push_back(deviceList->getDevice(i));
    }
    std::cout << "Devices found:" << std::endl;
    printDeviceList();

    // Callback function for device firmware update progress
    ob::Device::DeviceFwUpdateCallback firmwareUpdateCallback = [](OBFwUpdateState state, const char *message, uint8_t percent) {
        if(firstCall) {
            firstCall = !firstCall;
        }
        else {
            std::cout << "\033[3F"; // Move cursor up 3 lines
        }

        std::cout << "\033[K";  // Clear the current line
        std::cout << "Progress: " << static_cast<uint32_t>(percent) << "%" << std::endl;

        std::cout << "\033[K";
        std::cout << "Status  : ";
        switch(state) {
        case STAT_VERIFY_SUCCESS:
            std::cout << "Image file verification success" << std::endl;
            break;
        case STAT_FILE_TRANSFER:
            std::cout << "File transfer in progress" << std::endl;
            break;
        case STAT_DONE:
            std::cout << "Update completed" << std::endl;
            break;
        case STAT_IN_PROGRESS:
            std::cout << "Upgrade in progress" << std::endl;
            break;
        case STAT_START:
            std::cout << "Starting the upgrade" << std::endl;
            break;
        case STAT_VERIFY_IMAGE:
            std::cout << "Verifying image file" << std::endl;
            break;
        default:
            std::cout << "Unknown status or error" << std::endl;
            break;
        }

        std::cout << "\033[K";
        std::cout << "Message : " << message << std::endl << std::flush;
    };

    while(true) {
        firstCall = true;
        std::string firmwarePath;
        std::string input;
        int deviceIndex = -1;

        while (true) {
            std::cout << "Please select a device to update the firmware, enter 'l' or 'L' to list devices:" << std::endl;
            std::getline(std::cin, input);
            if (input == "l" || input == "L") {
                printDeviceList();
                continue;
            }

            try {
                deviceIndex = std::stoi(input); // exception
                if (deviceIndex < 0 || deviceIndex >= static_cast<int>(devices.size())) {
                    std::cout << "Invalid input, please enter a valid index number." << std::endl;
                    continue;
                }
            }
            catch(...) {
                std::cout << "Invalid input, please enter a valid index number." << std::endl;
                continue;
            }
        }

        if(!getFirmwarePath(firmwarePath)) {
            continue;
        }
        std::cout << "Upgrading device firmware, please wait...\n\n";

        try {
            // Set async to false to synchronously block and wait for the device firmware upgrade to complete.
            devices[deviceIndex]->updateFirmware(firmwarePath.c_str(), firmwareUpdateCallback, false);
        }
        catch(ob::Error &e) {
            // If the update fails, will throw an exception.
            std::cerr << "\nThe upgrade was interrupted! An error occurred! " << std::endl;
            std::cerr << "Error message: " << e.what() << std::endl;
            std::cout << "Press any key to exit." << std::endl;
            ob_smpl::waitForKeyPressed();
            break;
        }

        std::cout << "Enter 'Q' or 'q' to quit, or any other key to continue: ";
        std::getline(std::cin, input);
        if(input == "Q" || input == "q") {
            break;
        }
    }
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    std::cout << "\nPress any key to exit.";
    ob_smpl::waitForKeyPressed();
    exit(EXIT_FAILURE);
}

bool getFirmwarePath(std::string &firmwarePath) {
    std::cout << "Please input the path of the firmware file (.bin) to be updated:" << std::endl;
    std::cout << "Enter the path (or enter 'Q' or 'q' to quit): " << std::endl;
    std::string input;
    std::getline(std::cin, firmwarePath);

    if(input == "Q" || input == "q") {
        exit(EXIT_SUCCESS);
    }

    // Remove leading and trailing quotes
    if (!input.empty() && input.front() == '\'' && input.back() == '\'') {
        input = input.substr(1, input.size() - 2);
    }

    if (input.size() > 4 && input.substr(input.size() - 4) == ".bin") {
        firmwarePath = input;
        std::cout << "Firmware file confirmed: " << firmwarePath << std::endl;
        return true;
    } 

    std::cout << "Invalid file format. Please provide a .bin file." << std::endl;
    return false
}

void printDeviceList() {
    for (uint32_t i = 0; i < devices.size(); ++i) {
        std::cout << "[" << i << "] " << "Device: " << device->getDeviceInfo()->getName();
        std::cout << " serial number: " << device->getDeviceInfo()->getSerial();
        std::cout << " firmware version: " << device->getDeviceInfo()->getFirmwareVersion() << std::endl;
    }
}