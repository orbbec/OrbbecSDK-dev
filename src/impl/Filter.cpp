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

const char *ob_filter_get_vendor_specific_code(const char *name, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(name);
    auto filterFactory        = libobsensor::FilterFactory::getInstance();
    auto filterCreator        = filterFactory->getFilterCreator(name);
    auto privateFilterCreator = std::dynamic_pointer_cast<libobsensor::IPrivFilterCreator>(filterCreator);
    if(privateFilterCreator == nullptr) {
        return nullptr;
    }
    auto code = privateFilterCreator->getVendorSpecificCode();
    return code.c_str();
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, name)

ob_filter *ob_create_private_filter(const char *name, const char *activation_key, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(name);
    VALIDATE_NOT_NULL(activation_key);
    auto filterFactory = libobsensor::FilterFactory::getInstance();
    auto filter        = filterFactory->createPrivateFilter(name, activation_key);
    auto filterImpl    = new ob_filter();
    filterImpl->filter = filter;
    return filterImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, name, activation_key)

void ob_delete_filter(ob_filter *filter, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(filter);
    delete filter;
}
HANDLE_EXCEPTIONS_NO_RETURN(filter)

const char *ob_filter_get_config_schema(const ob_filter *filter, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(filter);
    auto& configSchema = filter->filter->getConfigSchema();
    return configSchema.c_str();
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, filter)

void ob_filter_update_config(ob_filter *filter, size_t argc, const char **argv, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(filter);
    std::vector<std::string> args(argv, argv + argc);
    filter->filter->updateConfig(args);
}
HANDLE_EXCEPTIONS_NO_RETURN(filter, argc, argv)

#ifdef __cplusplus
}
#endif