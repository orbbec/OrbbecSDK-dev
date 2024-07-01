#include <stdio.h>
#include <stdlib.h>
#include <libobsensor/ObSensor.h>
#include <libobsensor/ObSensor.h>

#include "utils.hpp" //todo

float color_count = 0;
float depth_count = 0;

uint64_t color_timestamp_last = 0;
uint64_t depth_timestamp_last = 0;

uint64_t color_timestamp_current = 0;
uint64_t depth_timestamp_current = 0;

int main(void) {

    // Used to return SDK interface error information.
    ob_error *error = NULL;

    // Create a pipeline to open the Color and Depth streams after connecting the device.
    ob_pipeline *pipe = ob_create_pipeline(&error);
    CHECK_OB_ERROR_EXIT(&error);

    // Start Pipeline.
    ob_pipeline_start(pipe, &error);
    CHECK_OB_ERROR_EXIT(&error);

    // Main loop
    while(true) {  // Wait in a loop, and exit after the window receives the "ESC_KEY" key
        if(_kbhit() && _getch() == ESC) {
            break;
        }

        // Wait for up to 100ms for a frameset in blocking mode.
        ob_frame *frameset = ob_pipeline_wait_for_frameset(pipe, 100, &error);
        CHECK_OB_ERROR_EXIT(&error);

        // to do
        // if(frameset != NULL) {
        //     // Get color frame from frameset.
        //     const ob_frame *color_frame = ob_frameset_get_frame(frameset, OB_FRAME_COLOR, &error);
        //     CHECK_OB_ERROR_EXIT(&error);
        //     // Get depth frame from frameset.
        //     const ob_frame *depth_frame = ob_frameset_get_frame(frameset, OB_FRAME_DEPTH, &error);
        //     CHECK_OB_ERROR_EXIT(&error);

        //     if(color_frame != NULL) {
        //         color_count++;
        //         // Get timestamp from color frame.
        //         color_timestamp_current = get_current_timestamp_ms();
        //         uint64_t duration       = color_timestamp_current - color_timestamp_last;
        //         if(duration > 3000) {  // 3 seconds
        //             if(color_timestamp_last != 0) { //filter first print
        //                 uint64_t color_frame_index = ob_frame_get_index(color_frame, &error);
        //                 double    color_frame_rate  = color_count / (duration / 1000.0);
        //                 // Print index and rate of color frame.
        //                 printf("color frame index: %lld, rate: %.2f\n", color_frame_index, color_frame_rate);
        //                 color_count = 0;
        //             }
        //             color_timestamp_last = color_timestamp_current;
        //         }

        //         ob_delete_frame(color_frame, &error);
        //     }

        //     if(depth_frame != NULL) {
        //         depth_count++;
        //         // Get timestamp from depth frame.
        //         depth_timestamp_current = get_current_timestamp_ms();
        //         uint64_t duration       = depth_timestamp_current - depth_timestamp_last;
        //         if(duration > 3000) {  // 3 seconds
        //             if(color_timestamp_last != 0) {
        //                 uint64_t depth_frame_index = ob_frame_get_index(depth_frame, &error);
        //                 double    depth_frame_rate  = depth_count / (duration / 1000.0);
        //                 // Print index and rate of depth frame.
        //                 printf("depth frame index: %lld, rate: %.2f\n", depth_frame_index, depth_frame_rate);
        //                 depth_count = 0;
        //             }
        //             depth_timestamp_last = depth_timestamp_current;
        //         }
        //         ob_delete_frame(depth_frame, &error);
        //     }

        //     // delete frame.
        //     ob_delete_frame(frameset, &error);
        //     CHECK_OB_ERROR_EXIT(&error);
        // }
    }

    // Stop Pipeline
    ob_delete_pipeline(pipe, &error);
    CHECK_OB_ERROR_EXIT(&error);

    return 0;
}
