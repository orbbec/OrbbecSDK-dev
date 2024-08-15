# C++ Sample: 1.stream.infrared

## Overview

Use SDK to obtain multiple camera data streams and output them

### Knowledge

An IR sensor is a sensor that uses infrared light for detection and measurement
Infrared light is a type of electromagnetic wave with a wavelength longer than visible light and is usually invisible to the human eye (wavelengths between approximately 700nm and 1mm)

## code overview

1. Instantiate the pipeline and configure the output video stream in addition to imu data, such as depth, color, etc.

    ```cpp
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

            // exclude gyro and accel sensors.
            if(sensorType == OB_SENSOR_GYRO || sensorType == OB_SENSOR_ACCEL) {
                continue;
            }

            // enable the stream.
            config->enableStream(sensorType);
        }

        // Start the pipeline with config
        std::mutex                          frameMutex;
        std::shared_ptr<const ob::FrameSet> renderFrameSet;
        pipe.start(config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
            std::lock_guard<std::mutex> lock(frameMutex);
            renderFrameSet = frameSet;
        });
    ```

2. Instantiate the pipeline, configure IMU related information and start streaming

    ```cpp
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
            std::lock_guard<std::mutex> lockImu(imuFrameMutex);
            renderImuFrameSet = frameSet;
        });
    ```

## Run Sample

Press the Esc key in the window to exit the program.

### Result

![image](Image/DepthViewer.png)
