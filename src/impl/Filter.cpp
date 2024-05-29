#include "ImplTypes.hpp"
#include "filter/FilterFactory.hpp"
#include "openobsdk/h/Filter.h"

#ifdef __cplusplus
extern "C" {
#endif

ob_filter *ob_create_filter(const char *name, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(name);
    auto filterFactory = libobsensor::FilterFactory::getInstance();
    auto filter        = filterFactory->createFilter(name);
    auto filterImpl    = new ob_filter();
    filterImpl->filter = filter;
    return filterImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, name)

void ob_delete_filter(ob_filter *filter, ob_error **error)BEGIN_API_CALL {
    VALIDATE_NOT_NULL(filter);
    delete filter;
} HANDLE_EXCEPTIONS_NO_RETURN(filter)

#ifdef __cplusplus
}
#endif