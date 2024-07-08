#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"

#include <iostream>

int main() try {
    // Create a pipeline with default device.
    ob::Pipeline pipe;

    // Get the device from the pipeline.
    std::shared_ptr<ob::Device> device = pipe.getDevice();

    while(true){

        // Get preset list from device.
        std::shared_ptr<ob::DevicePresetList> presetLists = device->getAvailablePresetList();
        std::cout << "Available Presets:" << std::endl;
        for(uint32_t index = 0; index < presetLists->count(); index++) {
            // Print available preset name.
            std::cout << " - " << index << "." << presetLists->getName(index) << std::endl;
        }

        // Print current preset name.
        std::cout << "Current PresetName: " << device->getCurrentPresetName() << std::endl;

        std::cout << "Enter index of preset to load: ";

        // Select preset to load.
        int  inputOption = ob_smpl::getInputOption();
        auto presetName = presetLists->getName(inputOption);

        // Load preset.
        device->loadPreset(presetName);

        // Print current preset name.
        std::cout << "Current PresetName: " << device->getCurrentPresetName() << std::endl;
    }

    // Stop Pipeline.
    pipe.stop();

    printf("\nProgram ended successfully. Press any key to exit.");
    getchar();
    getchar();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
