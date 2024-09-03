# C++ Sample: 4.misc.metadata

## Overview

Use the SDK interface to get the frameSet, then get the frame from frameSet, print the value of the frame metadata and exit the program using the ESC_KEY key.

### Knowledge

Pipeline is a pipeline for processing data streams, providing multi-channel stream configuration, switching, frame aggregation, and frame synchronization functions

Frameset is a combination of different types of Frames

Metadata is used to describe the various properties and states of a frame

## Code overview

1. Create an ob::Pipeline object, and start the pipeline.

    ```cpp
    // Create a pipeline.
    ob::Pipeline pipe;

    // Start the pipeline with default config.
    // Modify the default configuration by the configuration file: "OrbbecSDKConfig.xml"
    pipe.start();
    ```

2. Get key input

    ```cpp
    //get key input
    inputWatchThread = std::thread(inputWatcher);
    inputWatchThread.detach();

    void inputWatcher() {
        char input = ob_smpl::waitForKeyPressed();
        if(input == ESC_KEY) {
            exit(EXIT_SUCCESS);
        }
    }
    ```

3. Get frameSet

    ```cpp
    // Wait for frameSet from the pipeline, the default timeout is 1000ms.
    auto frameSet   = pipe.waitForFrameset();
    ```

4. Get frame from frameSet

    ```cpp
    // Wait for frameSet from the pipeline, the default timeout is 1000ms.
    auto frameSet   = pipe.waitForFrameset();
   ```

5. Get metadata from frame

    ```cpp
    //get the metadata of the frame
    for(uint32_t j = 0; j < static_cast<uint32_t>(metadataCount); j++) {
        //if the frame has the metadata, get the metadata value
        if(frame->hasMetadata(static_cast<OBFrameMetadataType>(j))) {
            std::cout << "metadata type: " << static_cast<OBFrameMetadataType>(j)
                        << " metadata value: " << frame->getMetadataValue(static_cast<OBFrameMetadataType>(j)) << std::endl;
        }
    }
    ```

6. Stop pipeline

    ```cpp
    // Stop the Pipeline, no frame data will be generated
    pipe.stop();
    ```

## Run Sample

Press the Esc key in the window to exit the program.

### Result

![image](/docs/resource/metadata.png)
