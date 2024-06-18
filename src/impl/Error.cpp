#ifdef __cplusplus
extern "C" {
#endif

#include "openobsdk/h/Error.h"

ob_status ob_error_get_status(const ob_error *error) {
    return error ? error->status : OB_STATUS_OK;
}

const char *ob_error_get_message(const ob_error *error) {
    return error ? error->message : nullptr;
}

const char *ob_error_get_function(const ob_error *error) {
    return error ? error->function : nullptr;
}

const char *ob_error_get_args(const ob_error *error) {
    return error ? error->args : nullptr;
}

ob_exception_type ob_error_get_exception_type(const ob_error *error) {
    return error ? error->exception_type : OB_EXCEPTION_TYPE_UNKNOWN;
}

void ob_delete_error(const ob_error *error) {
    if(error) {
        delete error;
        error = nullptr;
    }
}

#ifdef __cplusplus
}
#endif