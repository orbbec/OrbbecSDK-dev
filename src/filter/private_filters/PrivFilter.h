#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "openobsdk/h/Frame.h"

struct ob_private_filter;  // Forward declaration

typedef const char (*ob_priv_filter_get_uid)(ob_filter *filter, ob_error **error);
typedef void (*ob_priv_filter_activate)(ob_filter *filter, const char *activation_key, ob_error **error);

typedef const char *(*ob_priv_filter_get_config_schema)(ob_filter *filter, ob_error **error);
typedef void (*ob_priv_filter_update_config)(ob_filter *filter, size_t argc, const char **argv, ob_error **error);

typedef void (*ob_priv_filter_reset)(ob_filter *filter, ob_error **error);
typedef void (*ob_priv_filter_enable)(ob_filter *filter, bool enable, ob_error **error);
typedef bool (*ob_priv_filter_is_enabled)(ob_filter *filter, ob_error **error);

typedef ob_frame *(*ob_priv_filter_process)(ob_filter *filter, ob_frame *frame, ob_error **error);

typedef void (*ob_priv_filter_destroy)(ob_private_filter *filter, ob_error **error);

struct ob_private_filter {
    ob_filter *filter;  // Pointer to the filter object

    ob_priv_filter_destroy destroy;  // Destroy the filter object

    ob_priv_filter_get_uid  get_uid;   // Get the unique identifier of the filter
    ob_priv_filter_activate activate;  // Activate the filter with the given activation key

    ob_priv_filter_reset             reset;              // Reset the filter to its initial state (stop threads, clear buffers, etc.)
    ob_priv_filter_enable            enable;             // Enable or disable the filter
    ob_priv_filter_is_enabled        is_enabled;         // Check if the filter is enabled
    ob_priv_filter_get_config_schema get_config_schema;  // Get the configuration schema of the filter
    ob_priv_filter_update_config     update_config;      // Update the filter's configuration
    ob_priv_filter_process           process;            // Process a frame through the filter
};

typedef size_t (*ob_private_filter_list_get_count)();                              // Get the number of filters available in the dynamic library
typedef const char *(*ob_private_filter_list_get_name)(size_t index);              // Get the name of the filter at the given index
typedef ob_private_filter *(*ob_private_filter_list_create_filter)(size_t index);  // Create a new filter object from the filter at the given index

#ifdef __cplusplus
}
#endif