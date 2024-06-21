#include <stdio.h>
#include <stdlib.h>

#include <openobsdk/ObSensor.h>

#include "utils.hpp"
#define ESC 27



// helper function to check for errors and exit if there is one
void check_ob_error(ob_error **err) {
    if(*err) {
        const char* error_message = ob_error_get_message(*err);
        fprintf(stderr, "Error: %s\n", error_message);
        ob_delete_error(*err);
        exit(-1);
    }
    *err = NULL;
}

int main(void) {
    // Used to return SDK interface error information.
    ob_error    *error    = NULL;


    ob_config *config = ob_create_config(&error);
    check_ob_error(&error);

    ob_config_enable_stream(config, OB_STREAM_DEPTH, &error);
    check_ob_error(&error);

    // pipeline, used to open the depth stream after connecting the device.
    ob_pipeline *pipeline = ob_create_pipeline(&error);
    check_ob_error(&error);

    // Start Pipeline.
    ob_pipeline_start(pipeline, &error);
    ob_pipeline_start_with_config(pipeline, config, &error);
    check_ob_error(&error);

    // Wait in a loop, exit after the window receives the "esc" key
    while(true) {

        if(_kbhit() && _getch() == ESC) {
            break;
        }
        // Wait for up to 100ms for a frameset in blocking mode.
        const ob_frame *frameset = ob_pipeline_wait_for_frameset(pipeline, 100, &error);
        check_ob_error(&error);

        if(frameset == NULL) {
            continue;
        }

        // Get the depth frame from frameset。
        const ob_frame *depth_frame = ob_frameset_get_depth_frame(frameset, &error);
        check_ob_error(&error);
        if(depth_frame != NULL) {
            // Get index from depth frame.
            uint64_t index = ob_frame_get_index(depth_frame, &error);
            check_ob_error(&error);
            // Get format from depth frame.
            ob_format format = ob_frame_get_format(depth_frame, &error);
            check_ob_error(&error);

            // for Y16 format depth frame, print the distance of the center pixel every 30 frames
            if(index % 30 == 0 && format == OB_FORMAT_Y16) {
                uint32_t width = ob_video_frame_get_width(depth_frame, &error);
                check_ob_error(&error);
                uint32_t height = ob_video_frame_get_height(depth_frame, &error);
                check_ob_error(&error);
                float scale = ob_depth_frame_get_value_scale(depth_frame, &error);
                check_ob_error(&error);
                uint16_t *data = (uint16_t *)ob_frame_get_data(depth_frame, &error);
                check_ob_error(&error);

                // pixel value multiplied by scale is the actual distance value in millimeters
                float center_distance = data[width * height / 2 + width / 2] * scale;

                // attention: if the distance is 0, it means that the depth camera cannot detect the object（may be out of detection range）
                printf("Facing an object %.2f mm away.\n", center_distance);
            }
            ob_delete_frame(depth_frame, &error);
        }
        ob_delete_frame(frameset, &error);
        check_ob_error(&error);
    };

    // stop the pipeline
    ob_pipeline_stop(pipeline, &error);
    check_ob_error(&error);

    return 0;
}
