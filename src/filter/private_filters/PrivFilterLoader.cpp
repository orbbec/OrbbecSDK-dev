
#include "PrivFilterLoader.hpp"
#include "PrivFilterCppWrapper.hpp"
#include "exception/ObException.hpp"
#include "logger/Logger.hpp"
#include "utils/FileUtils.hpp"
namespace libobsensor {

PrivFilterCreator::PrivFilterCreator(std::shared_ptr<PrivFilterLibHandle> libHandle, size_t index) : libHandle_(libHandle), index_(index) {}

std::shared_ptr<IFilter> PrivFilterCreator::create() {
    ob_error *error = nullptr;

    bool activated = libHandle_->is_activated(&error);
    if(error) {
        std::string errorMsg = "Check if private filter library is activated: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    if(!activated) {
        throw unsupported_operation_exception("Private filter library not activated");
    }

    auto privFilter = libHandle_->create_filter(index_, &error);
    if(error) {
        std::string errorMsg = "Failed to create private filter: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    auto filterName = libHandle_->get_filter_name(index_, &error);
    if(error) {
        std::string errorMsg = "Failed to get filter name: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    return std::make_shared<PrivFilterCppWrapper>(filterName, privFilter);
}

std::shared_ptr<IFilter> PrivFilterCreator::create(const std::string &activationKey) {
    ob_error *error = nullptr;

    bool activated = libHandle_->is_activated(&error);
    if(error) {
        std::string errorMsg = "Check if private filter library is activated: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    if(!activated) {
        libHandle_->activate(activationKey.c_str(), &error);
        if(error) {
            std::string errorMsg = "Failed to activate private filter library: " + std::string(error->message);
            delete error;
            throw unsupported_operation_exception(errorMsg);
        }
    }

    auto filterName = libHandle_->get_filter_name(index_, &error);
    if(error) {
        std::string errorMsg = "Failed to get filter name: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    auto privFilter = libHandle_->create_filter(index_, &error);
    if(error) {
        std::string errorMsg = "Failed to create private filter: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    return std::make_shared<PrivFilterCppWrapper>(filterName, privFilter);
}

std::string PrivFilterCreator::getVendorSpecificCode() const {
    ob_error *error = nullptr;
    auto      code   = libHandle_->get_vendor_specific_code(&error);
    if(error) {
        std::string errorMsg = "Failed to get filter UID: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }
    return code;
}

namespace PrivFilterCreatorLoader {

#define DEFAULT_PRIVATE_FILTERS_ROOT_DIR "./extention/filters/"
std::map<std::string, std::shared_ptr<IFilterCreator>> getCreators() {
    // todo: get filters root dir from config file
    std::string filtersRootDir = DEFAULT_PRIVATE_FILTERS_ROOT_DIR;

    std::map<std::string, std::shared_ptr<IFilterCreator>> filterCreators;

    auto load = [&filterCreators](const std::string &dir, const std::string &libName) {
        auto libHandle                      = std::make_shared<PrivFilterLibHandle>();
        libHandle->dir                      = dir;
        libHandle->libName                  = libName;
        libHandle->dylib                    = std::make_shared<dylib>(dir, libName);
        libHandle->get_filter_count         = libHandle->dylib->get_function<size_t(ob_error **)>("ob_get_filter_count");
        libHandle->get_filter_name          = libHandle->dylib->get_function<const char *(size_t, ob_error **)>("ob_get_filter_name");
        libHandle->create_filter            = libHandle->dylib->get_function<ob_priv_filter_context *(size_t, ob_error **)>("ob_create_filter");
        libHandle->get_vendor_specific_code = libHandle->dylib->get_function<const char *(ob_error **)>("ob_priv_filter_get_vendor_specific_code");
        libHandle->is_activated             = libHandle->dylib->get_function<bool(ob_error **)>("ob_priv_filter_is_activated");
        libHandle->activate                 = libHandle->dylib->get_function<bool(const char *, ob_error **)>("ob_priv_filter_activate");

        ob_error *error       = nullptr;
        auto      filterCount = libHandle->get_filter_count(&error);
        if(error) {
            std::string errorMsg = "Failed to get filter count: " + std::string(error->message);
            delete error;
            throw unsupported_operation_exception(errorMsg);
        }

        for(size_t i = 0; i < filterCount; i++) {
            auto filterName = libHandle->get_filter_name(i, &error);
            if(error) {
                std::string errorMsg = "Failed to get filter name: " + std::string(error->message);
                delete error;
                throw unsupported_operation_exception(errorMsg);
            }

            auto creator               = std::make_shared<PrivFilterCreator>(libHandle, i);
            filterCreators[filterName] = creator;
            LOG_INFO("Private filter creator created: {}", filterName);
        }
    };

    // filter library without dependencies files (single file), can be placed in the root directory
    utils::forEachFileInDirectory(filtersRootDir, [&](const std::string &fileName) { TRY_EXECUTE({ load(filtersRootDir, fileName); }); });

    // filter library with dependencies files (multiple files), should be placed in a sub directory
    utils::forEachSubDirInDirectory(filtersRootDir, [&](const std::string &folderName) {
        auto dir     = filtersRootDir + folderName + "/";
        auto libName = folderName;  // the filter lib file name should be the same as the folder name
        TRY_EXECUTE({ load(dir, libName); });
    });

    return filterCreators;
}

}  // namespace PrivFilterCreatorLoader
}  // namespace libobsensor
