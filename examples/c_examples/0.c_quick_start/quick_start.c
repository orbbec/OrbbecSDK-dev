#include <stdio.h>
#include <stdlib.h>

#include <openobsdk/h/Error.h>
#include <openobsdk/h/Frame.h>
#include <openobsdk/h/ObTypes.h>
#include <openobsdk/h/Pipeline.h>
#include <openobsdk/h/StreamProfile.h>
#include <openobsdk/h/Device.h>
#include <openobsdk/h/Sensor.h>

#include "utils.hpp"
#define ESC 27

static float color_count = 0;
static float depth_count = 0;

static uint64_t color_timestamp_last = 0;
static uint64_t depth_timestamp_last = 0;

static uint64_t color_timestamp_current = 0;
static uint64_t depth_timestamp_current = 0;

// helper function to check for errors and exit if there is one.
void check_ob_error(ob_error **err) {
    if(*err) {
        const char* error_message = ob_error_get_message(*err);
        ob_delete_error(*err);
        fprintf(stderr, "Error: %s\n", error_message);
        exit(-1);
    }
    *err = NULL;
}

int main(){
    ob_error    *error    = NULL;

    // Create a pipeline to open the Color and Depth streams after connecting the device.
    ob_pipeline * pipe = ob_create_pipeline(&error);
    check_ob_error(&error);

    // Start Pipeline.
    ob_pipeline_start(pipe, &error);
    check_ob_error(&error);

    // Main loop
    while(true) {  // Wait in a loop, and exit after the window receives the "ESC_KEY" key
        /*if(kbhit() && getch() == ESC) {
            break;
        }*/
        // Wait for up to 100ms for a frameset in blocking mode.
        ob_frame *frameset = ob_pipeline_wait_for_frameset(pipe, 100, &error);
        check_ob_error(&error);

        if(frameset != NULL) {
            // Get color frame from frameset.
            ob_frame *color_frame = ob_frameset_get_frame(frameset, OB_FRAME_COLOR, & error);
            check_ob_error(&error);
            // Get deoth frame from frameset.
            ob_frame *depth_frame = ob_frameset_get_frame(frameset, OB_FRAME_DEPTH, &error);
            check_ob_error(&error);

            if(color_frame != NULL) {
                color_count++;
                // Get timestamp from color frame.
                color_timestamp_current = ob_frame_get_timestamp_us(color_frame, &error);
                if(color_timestamp_current - color_timestamp_last > 3000){
                    // Print index and rate of color frame.
                    printf("color frame index: %lld, rate: %f\n", ob_frame_get_index(color_frame, &error), color_count / 3);
                    color_count = 0;
                    color_timestamp_last = color_timestamp_current;
                }
                ob_delete_frame(color_frame, &error);
            }

            if(depth_frame != NULL) {
                depth_count++;
                // Get timestamp from depth frame.
                depth_timestamp_current = ob_frame_get_timestamp_us(depth_frame, &error);
                if(depth_timestamp_current - depth_timestamp_last > 3000){
                    // Print index and rate of depth frame.
                    printf("depth frame index: %lld, rate: %f\n",ob_frame_get_index(depth_frame, &error), depth_count/3);
                    depth_count = 0;
                    depth_timestamp_last = depth_timestamp_current;
                }
                ob_delete_frame(depth_frame, &error);
            }

            // delete frame.
            ob_delete_frame(frameset, &error);
            check_ob_error(&error);
        }
    }

    // Stop Pipeline
    ob_delete_pipeline(pipe, &error);
    check_ob_error(&error);

    return 0;
}