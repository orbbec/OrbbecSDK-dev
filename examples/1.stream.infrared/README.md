# C++ Sample: 1.stream.infrared

## Overview

Use the SDK interface to obtain the camera IR stream and display it in the window

### Knowledge

An IR sensor is a sensor that uses infrared light for detection and measurement
Infrared light is a type of electromagnetic wave with a wavelength longer than visible light and is usually invisible to the human eye (wavelengths between approximately 700nm and 1mm)

## code overview

1. Instantiate the pipeline, configure IR sensor related information and open the IR stream

    ```cpp
        // Create a pipeline with default device.
        ob::Pipeline pipe;

        // Get the device from pipeline.
        std::shared_ptr<ob::Device> device = pipe.getDevice();

        // Get the sensor list from device.
        std::shared_ptr<ob::SensorList> sensorList = device->getSensorList();

        // Create a config for pipeline.
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

        for(uint32_t index = 0; index < sensorList->getCount(); index++) {
            // Query all supported infrared sensor type and enable the infrared stream.
            // For dual infrared device, enable the left and right infrared streams.
            // For single infrared device, enable the infrared stream.
            OBSensorType sensorType = sensorList->getSensorType(index);
            if(sensorType == OB_SENSOR_IR || sensorType == OB_SENSOR_IR_LEFT || sensorType == OB_SENSOR_IR_RIGHT) {
                // Enable the stream with specified profile;
                config->enableVideoStream(sensorType, OB_WIDTH_ANY, OB_HEIGHT_ANY, 30, OB_FORMAT_ANY);
            }
        }

        pipe.start(config);
    ```

2. Open the window and output the IR stream

    ```cpp
        ob_smpl::CVWindow win("Infrared", 1280, 720, ob_smpl::ARRANGE_ONE_ROW);
        while(win.run()) {
        // Wait for up to 100ms for a frameset in blocking mode.
            auto frameSet = pipe.waitForFrameset(100);
            if(frameSet == nullptr) {
                continue;
        }

        // Render a set of frame in the window.
        win.pushFramesToView(frameSet);
        }
    ```

## Run Sample

Press the Esc key in the window to exit the program.

### Result

![image](Image/DepthViewer.png)
