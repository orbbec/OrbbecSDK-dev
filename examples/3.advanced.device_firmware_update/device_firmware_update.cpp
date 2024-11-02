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

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

bool getFirmwarePath(std::string &firmwarePath);
void onDeviceConnected(const std::string &prompt, std::shared_ptr<ob::DeviceList> deviceList);
void getDevice(const std::shared_ptr<ob::Context> &context);
void listBinFiles(const std::string &path, std::vector<std::string> &entries);

static bool firstCall     = true;
static bool firstMainCall = true;
static bool rebootDone    = false;

std::condition_variable     cond;
std::mutex                  mutex;
std::shared_ptr<ob::Device> device = nullptr;

int main() try {
    // Create a context and set the device changed callback function
    std::shared_ptr<ob::Context> context = std::make_shared<ob::Context>();

    // Set the device changed callback function
    context->setDeviceChangedCallback([](std::shared_ptr<ob::DeviceList> removedDeviceList, std::shared_ptr<ob::DeviceList> addDeviceList) {
        onDeviceConnected("connected", addDeviceList);
        onDeviceConnected("disconnected", removedDeviceList);
    });

    // Callback function for device firmware update progress
    ob::Device::DeviceFwUpdateCallback firmwareUpdateCallback = [](OBFwUpdateState state, const char *message, uint8_t percent) {
        if(!firstCall) {
            firstCall = !firstCall;
            // Move cursor up 3 lines
            std::cout << "\033[3F";
        }
#ifndef _WIN32
        // Clear the screen in linux
        system("clear");
#endif
        // Clear and print the progress line
        std::cout << "\033[K";
        std::cout << "Progress: " << static_cast<uint32_t>(percent) << "%" << std::endl;

        // Clear and print the status line
        std::cout << "\033[K";
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
        default:
            // exception will be thrown if the state is not recognized or if there is an error
            std::cout << "Unknown status or error";
            break;
        }
        std::cout << std::endl;

        // Clear and print the message line
        std::cout << "\033[K";
        std::cout << "Message: " << message << std::endl;
        std::cout << std::flush;
    };

    getDevice(context);

    while(true) {
        firstCall = true;
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
            // Reboot the device to apply the firmware update
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
#if 0
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
#endif
    std::string              currentPath = ".";
    std::vector<std::string> entries;

    while(true) {
        // List all .bin files in the current directory
        listBinFiles(currentPath, entries);

        if(entries.empty()) {
            std::cerr << "No firmware (.bin) files found in the current directory." << std::endl;
            std::cout << "To retry, place the firmware file in the current directory." << std::endl;
            std::cout << "Enter 'Q' or 'q' to quit, or press any other key to retry: ";
        
            char input = ob_smpl::waitForKeyPressed();
            if(input == 'Q' || input == 'q') {
                exit(EXIT_SUCCESS);
            }
            else {
                continue;
            }
        }

        // Display all .bin files
        std::cout << "\nFound the following firmware files (.bin):" << std::endl;
        for(size_t i = 0; i < entries.size(); ++i) {
            std::cout << "  [" << i << "] " << entries[i] << std::endl;
        }

        std::cout << "\nEnter the corresponding index number to select the firmware file, or enter 'Q' to quit: ";
        std::string input;
        std::getline(std::cin, input);

        // User chooses to quit
        if(input == "Q" || input == "q") {
            exit(EXIT_SUCCESS);
        }

        // If input is an index, automatically select the corresponding file
        if(!input.empty() && std::all_of(input.begin(), input.end(), ::isdigit)) {
            int index = std::stoi(input);
            if(index >= 0 && index < static_cast<int>(entries.size())) {
                firmwarePath = currentPath + "/" + entries[index];
                std::cout << "Firmware file confirmed: " << firmwarePath << std::endl;
                return true;
            }
            else {
                std::cerr << "Error: Invalid index." << std::endl;
                continue;
            }
        }
        else {
            std::cerr << "Error: Invalid input, please enter a valid index number." << std::endl;
        }
    }
}

void getDevice(const std::shared_ptr<ob::Context> &context) {
    if(firstMainCall) {
        firstMainCall                              = false;
        std::shared_ptr<ob::DeviceList> deviceList = context->queryDeviceList();
        if(deviceList->getCount() == 0) {
            // Wait for device connection
            std::cout << "No device found. Please connect a device first." << std::endl;
            std::cout << "Waiting for device connection..." << std::endl;
            {
                std::unique_lock<std::mutex> mtx(mutex);
                cond.wait(mtx, [&]() { return rebootDone; });
            }
        }
        else {
            // Get the first device in the list
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
        // Get the DeviceList again to update the device pointer
        std::shared_ptr<ob::DeviceList> deviceList = context->queryDeviceList();
        device                                     = deviceList->getDevice(0);
        std::shared_ptr<ob::DeviceInfo> deviceInfo = device->getDeviceInfo();

        // Clear the screen
        std::cout << "\033[2J\033[H";
        std::cout << "Device reboot completed, Device name: " << deviceInfo->getName() << std::endl;
        std::cout << "Current device firmware version: " << deviceInfo->getFirmwareVersion() << std::endl;
    }
}

void listBinFiles(const std::string &path, std::vector<std::string> &entries) {
    entries.clear();

#ifdef _WIN32
    WIN32_FIND_DATA findFileData;
    HANDLE          hFind = FindFirstFile((path + "\\*.bin").c_str(), &findFileData);

    if(hFind == INVALID_HANDLE_VALUE) {
        std::cerr << "Can not find any .bin files in directory: " << path << std::endl;
        return;
    }

    do {
        std::string name = findFileData.cFileName;
        entries.push_back(name);
    } while(FindNextFile(hFind, &findFileData) != 0);

    FindClose(hFind);
#else
    DIR           *dir;
    struct dirent *ent;

    if((dir = opendir(path.c_str())) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            std::string name = ent->d_name;
            if(name.size() >= 4 && name.substr(name.size() - 4) == ".bin") {
                entries.push_back(name);
            }
        }
        closedir(dir);
    }
    else {
        std::cerr << "Can not open directory: " << path << std::endl;
    }
#endif

    std::sort(entries.begin(), entries.end());
}