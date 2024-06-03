﻿
#include "PrivFilterLoader.hpp"
#include "PrivFilterCppWrapper.hpp"
#include "exception/ObException.hpp"
#include "logger/Logger.hpp"
#include "utils/FileUtils.hpp"
namespace libobsensor {

PrivFilterCreator::PrivFilterCreator(std::shared_ptr<PrivFilterPackageContext> pkgCtx, size_t index) : pkgCtx_(pkgCtx), index_(index) {}

std::shared_ptr<IFilter> PrivFilterCreator::create() {
    ob_error *error = nullptr;

    bool activated = pkgCtx_->is_activated(&error);
    if(error) {
        std::string errorMsg = "Check if private filter library is activated: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    if(!activated) {
        throw unsupported_operation_exception("Private filter library not activated");
    }

    auto privFilterCtx = pkgCtx_->create_filter(index_, &error);

    if(error) {
        std::string errorMsg = "Failed to create private filter: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    auto filterName = pkgCtx_->get_filter_name(index_, &error);
    if(error) {
        std::string errorMsg = "Failed to get filter name: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    auto pkgCtx              = pkgCtx_;  // Pass the pkgCtx to the deleter of shared_ptr to control the lifetime
    auto privFilterCtxShared = std::shared_ptr<ob_priv_filter_context>(privFilterCtx, [filterName, pkgCtx](ob_priv_filter_context *ctx) {
        if(ctx) {
            ob_error *error = nullptr;
            ctx->destroy(ctx, &error);
            if(error) {
                LOG_WARN("Private filter {} destroyed failed: {}", filterName, error->message);
                delete error;
            }
            ctx = nullptr;
        }
    });
    return std::make_shared<PrivFilterCppWrapper>(filterName, privFilterCtxShared);
}

std::shared_ptr<IFilter> PrivFilterCreator::create(const std::string &activationKey) {
    ob_error *error = nullptr;

    bool activated = pkgCtx_->is_activated(&error);
    if(error) {
        std::string errorMsg = "Check if private filter library is activated: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    if(!activated) {
        pkgCtx_->activate(activationKey.c_str(), &error);
        if(error) {
            std::string errorMsg = "Failed to activate private filter library: " + std::string(error->message);
            delete error;
            throw unsupported_operation_exception(errorMsg);
        }
    }

    auto filterName = pkgCtx_->get_filter_name(index_, &error);
    if(error) {
        std::string errorMsg = "Failed to get filter name: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    auto privFilterCtx = pkgCtx_->create_filter(index_, &error);
    if(error) {
        std::string errorMsg = "Failed to create private filter: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }

    auto pkgCtx              = pkgCtx_;  // Pass the pkgCtx to the deleter of shared_ptr to control the lifetime
    auto privFilterCtxShared = std::shared_ptr<ob_priv_filter_context>(privFilterCtx, [filterName, pkgCtx](ob_priv_filter_context *ctx) {
        if(ctx) {
            ob_error *error = nullptr;
            ctx->destroy(ctx, &error);
            if(error) {
                LOG_WARN("Private filter {} destroyed failed: {}", filterName, error->message);
                delete error;
            }
            ctx = nullptr;
        }
    });
    return std::make_shared<PrivFilterCppWrapper>(filterName, privFilterCtxShared);
}

std::string PrivFilterCreator::getVendorSpecificCode() const {
    ob_error *error = nullptr;
    auto      code  = pkgCtx_->get_vendor_specific_code(&error);
    if(error) {
        std::string errorMsg = "Failed to get filter UID: " + std::string(error->message);
        delete error;
        throw unsupported_operation_exception(errorMsg);
    }
    return code;
}

namespace PrivFilterCreatorLoader {

#define DEFAULT_PRIVATE_FILTERS_ROOT_DIR "./filter/"
std::map<std::string, std::shared_ptr<IFilterCreator>> getCreators() {
    // todo: get filters root dir from config file
    std::string filtersRootDir = DEFAULT_PRIVATE_FILTERS_ROOT_DIR;

    std::map<std::string, std::shared_ptr<IFilterCreator>> filterCreators;

    auto load = [&filterCreators](const std::string &dir, const std::string &packageName) {
        auto pkgCtx_         = std::make_shared<PrivFilterPackageContext>();
        pkgCtx_->dir         = dir;
        pkgCtx_->packageName = packageName;
        auto fileName        = utils::removeExtensionOfFileName(packageName);
        try {
            pkgCtx_->dylib                    = std::make_shared<dylib>(dir, fileName);
            pkgCtx_->get_filter_count         = pkgCtx_->dylib->get_function<size_t(ob_error **)>("ob_get_filter_count");
            pkgCtx_->get_filter_name          = pkgCtx_->dylib->get_function<const char *(size_t, ob_error **)>("ob_get_filter_name");
            pkgCtx_->create_filter            = pkgCtx_->dylib->get_function<ob_priv_filter_context *(size_t, ob_error **)>("ob_create_filter");
            pkgCtx_->get_vendor_specific_code = pkgCtx_->dylib->get_function<const char *(ob_error **)>("ob_priv_filter_get_vendor_specific_code");
            pkgCtx_->is_activated             = pkgCtx_->dylib->get_function<bool(ob_error **)>("ob_priv_filter_is_activated");
            pkgCtx_->activate                 = pkgCtx_->dylib->get_function<bool(const char *, ob_error **)>("ob_priv_filter_activate");
        }
        catch(const std::exception &e) {
            LOG_DEBUG("Failed to load private filter library {}: {}", dir + packageName, e.what());
            return;
        }
        ob_error *error       = nullptr;
        auto      filterCount = pkgCtx_->get_filter_count(&error);
        if(error) {
            std::string errorMsg = "Failed to get filter count: " + std::string(error->message);
            delete error;
            throw unsupported_operation_exception(errorMsg);
        }

        for(size_t i = 0; i < filterCount; i++) {
            auto filterName = pkgCtx_->get_filter_name(i, &error);
            if(error) {
                std::string errorMsg = "Failed to get filter name: " + std::string(error->message);
                delete error;
                throw unsupported_operation_exception(errorMsg);
            }

            auto creator               = std::make_shared<PrivFilterCreator>(pkgCtx_, i);
            filterCreators[filterName] = creator;
            LOG_DEBUG("Private filter creator created: {}", filterName);
        }
    };

    // filter library without dependencies files (single file), can be placed in the root directory
    utils::forEachFileInDirectory(filtersRootDir, [&](const std::string &fileName) { TRY_EXECUTE({ load(filtersRootDir, fileName); }); });

    // filter library with dependencies files (multiple files), should be placed in a sub directory
    utils::forEachSubDirInDirectory(filtersRootDir, [&](const std::string &folderName) {
        auto dir         = filtersRootDir + folderName + "/";
        auto packageName = folderName;  // the filter lib file name should be the same as the folder name
        TRY_EXECUTE({ load(dir, packageName); });
    });

    return filterCreators;
}

}  // namespace PrivFilterCreatorLoader
}  // namespace libobsensor
