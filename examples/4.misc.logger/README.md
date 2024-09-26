# C++ Sample: 4.msic.loggger

## Overview
Use the SDK context class or modify the XML configuration file to set the SDK log level, including the log level output to the terminal and the log level output to the file.

### Knowledge
Context, which serves as the entry point to the underlying SDK. It is used to query device lists, handle device callbacks, and set the log level.

## Code Overview

1. Configure the log level output to the terminal

    ```cpp
    // Configure the log level output to the terminal.
    ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_ERROR);
    ```

2. Configure the log level output to the file.
The default output path settings(Windows/Linux) is `${CWD}/Log/OrbbecSDK.log`, where `${CWD}` represents the Current Working Directory. For Android, the path is `/sdcard/orbbec/Log`.

    ```cpp
    // Configure the log level and path output to the file.
    // The first parameter is the log level, and the second parameter is the output path.
    // If the output path is empty, the existing settings will continue to be used (if the existing configuration is also empty, the log will not be output to the file).
    ob::Context::setLoggerToFile(OB_LOG_SEVERITY_DEBUG, "Log/Custom/");
    ```

3. Registering a log callback

    ```cpp
    // Register a log callback, you can get log information in the callback.
    // The first parameter is the log level, and the second parameter is the callback function.
    ob::Context::setLoggerToCallback(OB_LOG_SEVERITY_DEBUG, [](OBLogSeverity severity, const char *logMsg) {
        std::cout << "[CallbackMessage][Level:" << severity << "]" << logMsg;
    });
    ```

## Configuration via XML

The log level can also be configured via XML. You can find the configuration file in the path `OrbbecSDK-dev/src/shared/environment/OrbbecSDKConfig.xml`. Then find the location of the Log node you will see the following.

```xml
    <Log>
        <!-- Log output level, int type, optional values: 0-DEBUG, 1-INFO, 2-WARN, 3-ERROR, 4-FATAL,
        5-OFF -->
        <!-- File log output level -->
        <FileLogLevel>5</FileLogLevel>
        <!-- Console log output level -->
        <ConsoleLogLevel>3</ConsoleLogLevel>
        <!-- Default log output file path, string type. If this item is not configured, the default
        path will be used: Win/Linux: "./Log"; Android: "/sdcard/orbbec/Log" -->
        <!-- <OutputDir>./log</OutputDir> -->
        <!-- Default log output file size, int type, unit: MB -->
        <MaxFileSize>100</MaxFileSize>
        <!-- Default log output file number (circular overwrite), int type -->
        <MaxFileNum>3</MaxFileNum>
        <!-- Log asynchronous output, changing to asynchronous output can reduce the blocking time
        of printing logs, but some logs may be lost when the program exits abnormally; true-enable,
        false-disable (default) -->
        <Async>false</Async>
    </Log>
```
You can modify the value of the `MaxFileSize` node to control the maximum size of a single file log; by modifying the value of the `MaxFileNum` node, you can control the maximum number of logs generated.

## Run Sample
If you are on Windows, you can switch to the directory `OrbbecSDK-dev/build/win_XX/bin` to find the `ob_logger.exe`.

If you are on linux, you can switch to the directory `OrbbecSDK-dev/build/linux_XX/bin` to find the `ob_logger`.

### Result
![result](/docs/resource/logger.jpg)
