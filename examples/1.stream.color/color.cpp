#include "window.hpp"

#include <openobsdk/ObSensor.h>

const char* metaDataTypes[] = {
    "OB_FRAME_METADATA_TYPE_TIMESTAMP",
    "OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP",
    "OB_FRAME_METADATA_TYPE_FRAME_NUMBER",
    "OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE",
    "OB_FRAME_METADATA_TYPE_EXPOSURE",
    "OB_FRAME_METADATA_TYPE_GAIN",
    "OB_FRAME_METADATA_TYPE_AUTO_WHITE_BALANCE",
    "OB_FRAME_METADATA_TYPE_WHITE_BALANCE",
    "OB_FRAME_METADATA_TYPE_BRIGHTNESS",
    "OB_FRAME_METADATA_TYPE_CONTRAST",
    "OB_FRAME_METADATA_TYPE_SATURATION",
    "OB_FRAME_METADATA_TYPE_SHARPNESS",
    "OB_FRAME_METADATA_TYPE_BACKLIGHT_COMPENSATION",
    "OB_FRAME_METADATA_TYPE_HUE",
    "OB_FRAME_METADATA_TYPE_GAMMA",
    "OB_FRAME_METADATA_TYPE_POWER_LINE_FREQUENCY",
    "OB_FRAME_METADATA_TYPE_LOW_LIGHT_COMPENSATION",
    "OB_FRAME_METADATA_TYPE_MANUAL_WHITE_BALANCE",
    "OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE",
    "OB_FRAME_METADATA_TYPE_FRAME_RATE",
    "OB_FRAME_METADATA_TYPE_AE_ROI_LEFT",
    "OB_FRAME_METADATA_TYPE_AE_ROI_TOP",
    "OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT",
    "OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM",
    "OB_FRAME_METADATA_TYPE_EXPOSURE_PRIORITY",
    "OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_NAME",
    "OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_SIZE",
    "OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX",
    "OB_FRAME_METADATA_TYPE_LASER_POWER",
    "OB_FRAME_METADATA_TYPE_LASER_POWER_MODE",
    "OB_FRAME_METADATA_TYPE_EMITTER_MODE",
    "OB_FRAME_METADATA_TYPE_GPIO_INPUT_DATA"
};

int main(int argc, char **argv) try {

    // Create a pipeline with default device.
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config.
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Enable color video stream.
    config->enableVideoStream(OB_STREAM_COLOR);

    // Start the pipeline with config.
    pipe.start(config);

    // Create a window for rendering and set the resolution of the window.
    Window app("ColorViewer", 1280, 720);
    // Metadata refresh frequency.
    int interval = 0;
    while(app) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames();
        if(frameSet == nullptr) {
            continue;
        }
        if(frameSet!=nullptr){
            // get color frame from frameset.
            auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
            interval--;
            if(interval <= 0){
                interval = 100;
                std::cout << "MetaData List: " << std::endl;
                for(int metaDataType = 0; metaDataType < sizeof(metaDataTypes) / sizeof(metaDataTypes[0]); metaDataType++){
                    // Check if it is supported metaDataType.
                    if(colorFrame->hasMetadata((OBFrameMetadataType)metaDataType)){
                        // get metaData value.
                        std::cout << " - " <<metaDataTypes[metaDataType] << "-->" << colorFrame->getMetadataValue((OBFrameMetadataType)metaDataType) << std::endl;
                    }else{
                        std::cout << " - " << metaDataTypes[metaDataType] << "-->" << "unsupport" << std::endl;
                    }
                }
            }
        }

        // Render frameset in the window, only color frames are rendered here.
        app.addToRender(frameSet->getFrame(OB_FRAME_COLOR));
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
