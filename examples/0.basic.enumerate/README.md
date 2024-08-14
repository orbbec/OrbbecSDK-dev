# C++ Sample Post Processing

## Overview

Use the SDK interface to obtain camera-related information, including model, various sensors, and sensor-related configurations.

### Knowledge

Context is the environment context, the first object created during initialization, which can be used to perform some settings, including but not limited to device status change callbacks, log level settings, etc. Context can access multiple Devices.

Device is the device object, which can be used to obtain the device information, such as the model, serial number, and various sensors.One actual hardware device corresponds to one Device object.



## Code overview





















## 1. Get the pipeline and configure the stream

```cpp
    // Create a pipeline with default device
    ob::Pipeline pipe;
    
    // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
    auto profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    
    std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
    try {
        // Find the corresponding profile according to the specified format, first look for the y16 format
        depthProfile = profiles->getVideoStreamProfile(640, OB_HEIGHT_ANY, OB_FORMAT_Y16, 30);
    }
    catch(ob::Error &e) {
        // If the specified format is not found, search for the default profile to open the stream
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(profiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
    }
    
    // By creating config to configure which streams to enable or disable for the pipeline, here the depth stream will be enabled
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(depthProfile);
```

## 2. Get a list of depth post-processing filters
```cpp 
    auto obFilterList = pipe.getDevice()->getSensor(OB_SENSOR_DEPTH)->getRecommendedFilters();
    
    std::shared_ptr<ob::DecimationFilter> decFilter;
    for(int i = 0; i < obFilterList->count(); i++) {
        auto postProcessorfilter =obFilterList->getFilter(i);
        std::cout << "Depth recommended post processor filter type: " << postProcessorfilter->type() << std::endl;
        if(postProcessorfilter->is<ob::DecimationFilter>()) {
            decFilter = postProcessorfilter->as<ob::DecimationFilter>();
        }
    }
```

## 3. Start pipeline
```cpp
    pipe.start(config);
```

## 4. expected Output 

![image](Image/post_processing.png)