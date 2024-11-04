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
#include <cstdlib>
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

    getDevice(context);

    while(true) {
        firstCall = true;
        std::string firmwarePath;
        if(!getFirmwarePath(firmwarePath)) {
            continue;
        }
        std::cout << "Upgrading device firmware, please wait...\n\n";

        try {
            // Set async to false to synchronously block and wait for the device firmware upgrade to complete.
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
        std::cout << "  [R / r]         - Reboot the device to apply the firmware update\n";
        std::cout << "  [Q / q]         - Quit the program\n";
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
    std::string              currentPath = ".";
    std::vector<std::string> entries;

    while(true) {
        // List all .bin files in the current directory
        listBinFiles(currentPath, entries);

        if(entries.empty()) {
            std::cerr << "Error: No firmware (.bin) files found in the current directory." << std::endl;
            std::cout << "Please follow these steps:" << std::endl;
            std::cout << "  1. Place the (.bin) file in the current program directory." << std::endl;
            std::cout << "  2. After completing step 1, enter any key to retry." << std::endl;
            std::cout << "      Or, enter 'Q' or 'q' to quit." << std::endl;
            std::cout << "==================================================================" << std::endl;

            std::string input;
            std::getline(std::cin, input);
            if(input == "Q" || input == "q") {
                exit(EXIT_SUCCESS);
            }
            else {
                continue;
            }
        }

        // Display all .bin files
        std::cout << "Found the following firmware files (.bin):" << std::endl;
        for(size_t i = 0; i < entries.size(); ++i) {
            std::cout << "  [" << i << "] " << entries[i] << std::endl;
        }

        std::cout << "\nEnter the corresponding index number to select the firmware file, or enter 'Q' or 'q' to quit: ";
        std::string input;
        std::getline(std::cin, input);

        if(input == "Q" || input == "q") {
            exit(EXIT_SUCCESS);
        }

        try {
            if(!input.empty() && std::all_of(input.begin(), input.end(), ::isdigit)) {
                int index = std::stoi(input); // exception
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
        catch(...) {
            std::cerr << "Error: Invalid input, please enter a valid index number." << std::endl;
        }
    }
}

void getDevice(const std::shared_ptr<ob::Context> &context) {
    if(firstMainCall) {
        firstMainCall                              = !firstMainCall;
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
            std::cout << "Current device firmware version: " << device->getDeviceInfo()->getFirmwareVersion() << std::endl << std::endl;
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

        std::cout << "\033[2J\033[H";  // Clear the screen
        std::cout << "Device reboot completed, Device name: " << deviceInfo->getName() << std::endl;
        std::cout << "Current device firmware version: " << deviceInfo->getFirmwareVersion() << std::endl << std::endl;
    }
}

void listBinFiles(const std::string &path, std::vector<std::string> &entries) {
    entries.clear();

#ifdef _WIN32
    WIN32_FIND_DATA findFileData;
    HANDLE          hFind = FindFirstFileW((std::wstring(path.begin(), path.end()) + L"\\*.bin").c_str(), &findFileData);

    if(hFind == INVALID_HANDLE_VALUE) {
        return;
    }

    do {
        std::wstring wname = findFileData.cFileName;
        size_t       len   = wname.length();
        std::string  name(len, '\0');
        std::wcstombs(&name[0], wname.c_str(), len);
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