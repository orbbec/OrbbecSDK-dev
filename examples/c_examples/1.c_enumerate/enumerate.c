#include <stdio.h>
#include <stdlib.h>

#include <openobsdk/ObSensor.h>

const char* sensor_types[] =  {
    "OB_SENSOR_UNKNOWN",
    "OB_SENSOR_IR",
    "OB_SENSOR_COLOR",
    "OB_SENSOR_DEPTH",
    "OB_SENSOR_ACCEL",
    "OB_SENSOR_GYRO",
    "OB_SENSOR_IR_LEFT",
    "OB_SENSOR_IR_RIGHT",
    "OB_SENSOR_RAW_PHASE",
    "OB_SENSOR_COUNT"
};

const char *stream_types[] = { "OB_FORMAT_YUYV",  "OB_FORMAT_YUY2",       "OB_FORMAT_UYVY", "OB_FORMAT_NV12",   "OB_FORMAT_NV21",   "OB_FORMAT_MJPG",
                              "OB_FORMAT_H264",  "OB_FORMAT_H265",       "OB_FORMAT_Y16",  "OB_FORMAT_Y8",     "OB_FORMAT_Y10",    "OB_FORMAT_Y11",
                              "OB_FORMAT_Y12",   "OB_FORMAT_GRAY",       "OB_FORMAT_HEVC", "OB_FORMAT_I420",   "OB_FORMAT_ACCEL",  "OB_FORMAT_GYRO",
                              "OB_FORMAT_POINT", "OB_FORMAT_RGB_POINT",  "OB_FORMAT_RLE",  "OB_FORMAT_RGB",    "OB_FORMAT_BGR",    "OB_FORMAT_Y14",
                              "OB_FORMAT_BGRA",  "OB_FORMAT_COMPRESSED", "OB_FORMAT_RVL",  "OB_FORMAT_Z16",    "OB_FORMAT_YV12",   "OB_FORMAT_BA81",
                              "OB_FORMAT_RGBA",  "OB_FORMAT_BYR2",       "OB_FORMAT_RW16", "OB_FORMAT_DISP16", "OB_FORMAT_UNKNOWN" };

const char *rate_types[] = {
    "OB_SAMPLE_RATE_UNKNOWN",
    "OB_SAMPLE_RATE_1_5625_HZ",
    "OB_SAMPLE_RATE_3_125_HZ",
    "OB_SAMPLE_RATE_6_25_HZ",
    "OB_SAMPLE_RATE_12_5_HZ",
    "OB_SAMPLE_RATE_25_HZ",
    "OB_SAMPLE_RATE_50_HZ",
    "OB_SAMPLE_RATE_100_HZ",
    "OB_SAMPLE_RATE_200_HZ",
    "OB_SAMPLE_RATE_500_HZ",
    "OB_SAMPLE_RATE_1_KHZ",
    "OB_SAMPLE_RATE_2_KHZ",
    "OB_SAMPLE_RATE_4_KHZ",
    "OB_SAMPLE_RATE_8_KHZ",
    "OB_SAMPLE_RATE_16_KHZ",
    "OB_SAMPLE_RATE_32_KHZ",
};

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

// Enumerates stream information.
void enumerate_stream_info(ob_sensor *sensor, ob_error **error){
    // Get sensor type.
    ob_sensor_type sensor_type = ob_sensor_get_type(sensor, error);
    // Get stream profile list.
    ob_stream_profile_list * stream_profile_list = ob_sensor_get_stream_profile_list(sensor, error);
    // Get stream profile count.
    uint32_t stream_profile_count = ob_stream_profile_list_count(stream_profile_list, error);

    for(uint32_t index=0; index < stream_profile_count; index++){
        // Get stream profile.
        ob_stream_profile *stream_profile = ob_stream_profile_list_get_profile(stream_profile_list, index, error);
        // Print video stream profile information.
        if(sensor_type == OB_SENSOR_IR || sensor_type == OB_SENSOR_COLOR || sensor_type == OB_SENSOR_DEPTH || sensor_type == OB_SENSOR_IR_LEFT || sensor_type == OB_SENSOR_IR_RIGHT){
            ob_format stream_format    = ob_stream_profile_get_format(stream_profile, error);
            uint32_t stream_width = ob_video_stream_profile_get_width(stream_profile, error);
            uint32_t stream_height = ob_video_stream_profile_get_height(stream_profile, error);
            uint32_t stream_fps = ob_video_stream_profile_get_fps(stream_profile, error);
            printf(" - type:%s, width:%d, height:%d, fps:%d\n", stream_types[stream_format], stream_width, stream_height, stream_fps);
        }else if(sensor_type == OB_SENSOR_ACCEL){
            // Print acc stream profile information.
            ob_format stream_format    = ob_stream_profile_get_format(stream_profile, error);
            ob_accel_sample_rate acc_fps = ob_accel_stream_profile_get_sample_rate(stream_profile, error);
            printf(" - type:%s, fps:%s\n", stream_types[stream_format], rate_types[acc_fps]);
        }else if(sensor_type == OB_SENSOR_GYRO){
            // Print gyro stream profile information.
            ob_format stream_format    = ob_stream_profile_get_format(stream_profile, error);
            ob_gyro_sample_rate gyro_fps = ob_gyro_stream_profile_get_sample_rate(stream_profile, error);
            printf(" - type:%s, fps:%s\n", stream_types[stream_format], rate_types[gyro_fps]);
        }
    }

}

// Enumerates device information.
void enumerate_device_info(ob_device *device,int index, ob_error **error){
    // Get device information.
    ob_device_info * dev_inf = ob_device_get_device_info(device, error);
    // Get device name.
    const char *dev_name = ob_device_info_get_name(dev_inf, error);
    // Get device pid.
    int dev_pid = ob_device_info_get_pid(dev_inf, error);
    // Get device serial number.
    const char * dev_sn = ob_device_info_get_serial_number(dev_inf, error);
    printf("%d - device name : %s, device pid : %d, device sn : %s\n",index, dev_name, dev_pid, dev_sn);
}

// enumerate sensor list.
void enumerate_sensor_info(ob_sensor_list * sensor_list, ob_error **error){
    while(true){
        // Get sensor count.
        uint32_t sensor_count = ob_sensor_list_get_sensor_count(sensor_list, error);
        for(uint32_t index=0; index < sensor_count; index++){
            // Get device sonsor.
            ob_sensor *sensor = ob_sensor_list_get_sensor(sensor_list, index, error);
            // Get sensor type.
            ob_sensor_type sensor_name = ob_sensor_get_type(sensor, error);
            printf("%d - sensor name : %s\n",index, sensor_types[sensor_name]);
        }
        printf("Select a sensor to enumerate its streams(input sensor index or \'q\' to enumerate device): \n");
        // Select sensor.
        char sensor_selected;
        scanf("%c", &sensor_selected);
        getchar();
        if(sensor_selected == 'q' || sensor_selected == 'Q'){
            return -1;
        }
        enumerate_stream_info(ob_sensor_list_get_sensor(sensor_list, sensor_selected-'0', error), error);
    }
}

int main(){

    ob_error    *error = NULL;
    // Set logger severity.
    ob_set_logger_severity(OB_LOG_SEVERITY_ERROR, &error);
    check_ob_error(&error);

    // Creat context.
    ob_context *ctx   = ob_create_context(&error);
    check_ob_error(&error);

    while(true){
        // Get device list from context.
        ob_device_list *dev_list = ob_query_device_list(ctx, &error);
        check_ob_error(&error);
        // Get device count from device list.
        int dev_count = ob_device_list_get_device_count(dev_list, &error);
        check_ob_error(&error);
        if(dev_count == 0) {
            printf("Device not found!\n");
            return -1;
        }

        printf("enumerated devices: \n");
        for(int index=0; index < dev_count; index++){
            // Get device from device list.
            ob_device *dev = ob_device_list_get_device(dev_list, index, &error);
            check_ob_error(&error);
            enumerate_device_info(dev, index, &error);
        }

        printf("enumerate sensors of device (input device index or \'q\' to exit program):\n");

        // Select a device.
        char device_selected;
        scanf("%c", &device_selected);
        getchar();
        if(device_selected == 'q' || device_selected == 'Q'){
            return -1;
        }

        // Get sensor list from device list.
        ob_sensor_list * sensor_list = ob_device_get_sensor_list(ob_device_list_get_device(dev_list, device_selected-'0', &error), &error);
        enumerate_sensor_info(sensor_list, &error);
    }

    // destroy context
    ob_delete_context(ctx, &error);
    check_ob_error(&error);

    return 0;
}