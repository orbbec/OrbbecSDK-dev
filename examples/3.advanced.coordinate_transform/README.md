# C++ Sample Coordinate Transform

Supported devices: Gemini 330 series cameras, such as Gemini G335

Function description: The example uses tools to transform different coordinate systems

This example is based on the C++ high level API for demonstration

## 1. Create pipeline

```cpp
    ob::Pipeline pipe;
```

## 2. Enable color stream

```cpp
    auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
    if(colorProfiles) {
        colorProfile = colorProfiles->getVideoStreamProfile(1280, OB_HEIGHT_ANY, OB_FORMAT_RGB, 30);
    }
    config->enableStream(colorProfile);
```

## 3. Enable depth stream

```cpp
    auto                                    depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
    if(depthProfiles) {
    depthProfile = depthProfiles->getVideoStreamProfile(640, OB_HEIGHT_ANY, OB_FORMAT_Y16, 30);

    }
    config->enableStream(depthProfile);
```

## 4. Start Pipeline through Configuration

```cpp
    pipe.start(config);
```

## 5. Get frame data

```cpp
    auto colorFrame = frameSet->colorFrame();
    auto depthFrame = frameSet->depthFrame();
```

## 6.Get get stream profile

```cpp
    auto colorProfile =  colorFrame->getStreamProfile();
    auto depthProfile = depthFrame->getStreamProfile();
```

## 7.Get the extrinsic parameters

```cpp
    auto extrinsicD2C = depthProfile->getExtrinsicTo(colorProfile);
    auto extrinsicC2D = colorProfile->getExtrinsicTo(depthProfile);
```

## 8.Get the intrinsic parameters

```cpp
    auto colorIntrinsic = colorProfile->as<ob::VideoStreamProfile>()->getIntrinsic();
    auto colorDistortion = colorProfile->as<ob::VideoStreamProfile>()->getDistortion();
```

## 9.Get the distortion parameters

```cpp
    auto depthIntrinsic = depthProfile->as<ob::VideoStreamProfile>()->getIntrinsic();
    auto depthDistortion = depthProfile->as<ob::VideoStreamProfile>()->getDistortion();
```

## 10. Processing

```cpp
     if(testType == "1") {
            transformation2dto2d(colorFrame, depthFrame);
        } else if (testType == "2") {
            transformation2dto3d(colorFrame, depthFrame);
        } else if (testType == "3") {
            transformation3dto3d(colorFrame, depthFrame);
        } else if (testType == "4") {
            transformation3dto2d(colorFrame, depthFrame);
        } else {
            std::cout << "Invalid command" << std::endl;
        }  
```

## 8. Stop pipeline

```cpp
    pipe.stop();
```
