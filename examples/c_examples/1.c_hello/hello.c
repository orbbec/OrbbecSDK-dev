#include <stdio.h>
#include <stdlib.h>

#include <openobsdk/h/Context.h>
#include <openobsdk/h/Error.h>

// helper function to check for errors and exit if there is one
void check_ob_error(ob_error **err) {
    if(*err) {
        const char* error_message = ob_error_get_message(*err);
        ob_delete_error(*err);
        fprintf(stderr, "Error: %s\n", error_message);
        exit(-1);
    }
    *err = NULL;
}

// void printf_device_list(ob_device_list *device_list) {

// }

// void printf_device_info(ob_device_info *device_info) {

// }

int main() {
    // define the error pointer to handle errors
    ob_error* err = NULL;

    // set the logger to console with debug severity
    ob_set_logger_to_console(OB_LOG_SEVERITY_DEBUG, &err);
    check_ob_error(&err);

    // create a context
    ob_context* ctx = ob_create_context(&err);
    check_ob_error(&err);

    while(true){}

    // ob_device_list *device_list = ob_query_device_list(ctx, &err);
    // check_ob_error(&err);

    // printf_device_list(device_list);

    // todo：create device and print device info
    // todo： delete device_list, device, etc.

    // delete the context
    ob_delete_context(ctx, &err);
    check_ob_error(&err);

}