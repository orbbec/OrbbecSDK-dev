/**
 * @file  Filter.hpp
 * @brief This file contains the Filter class, which is the processing unit of the SDK that can perform point cloud generation, format conversion, and other
 * functions.
 */
#pragma once

#include "Types.hpp"
#include "Error.hpp"
#include "Frame.hpp"
#include "libobsensor/h/Filter.h"
#include "libobsensor/h/Frame.h"
#include <functional>
#include <memory>
#include <map>
#include <string>
#include <iostream>

namespace ob {

/**
 * @brief A callback function that takes a shared pointer to a Frame object as its argument.
 */
typedef std::function<void(std::shared_ptr<Frame>)> FilterCallback;

/**
 * @brief The Filter class is the base class for all filters in the SDK.
 */
class Filter : public std::enable_shared_from_this<Filter> {
protected:
    ob_filter     *impl_ = nullptr;
    std::string    name_;
    FilterCallback callback_;

public:
    Filter() = default;
    Filter(ob_filter *impl) : impl_(impl) {
        ob_error *error = nullptr;
        name_           = ob_filter_get_name(impl_, &error);
        Error::handle(&error);
    }

    virtual ~Filter() noexcept {
        if(impl_ != nullptr) {
            ob_error *error = nullptr;
            ob_delete_filter(impl_, &error);
            Error::handle(&error, false);
        }
    }

    /**
     * @brief Get the type of filter.
     *
     * @return string The type of filte.
     */
    virtual const std::string &getName() {
        return name_;
    }

    /**
     * @brief Reset the filter, freeing the internal cache, stopping the processing thread, and clearing the pending buffer frame when asynchronous processing
     * is used.
     */
    virtual void reset() {
        ob_error *error = nullptr;
        ob_filter_reset(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief enable the filter
     */
    virtual void enable(bool enable) {
        ob_error *error = nullptr;
        ob_filter_enable(impl_, enable, &error);
        Error::handle(&error);
    }

    /**
     * @brief Return Enable State
     */
    virtual bool isEnabled() {
        ob_error *error  = nullptr;
        bool      enable = ob_filter_is_enabled(impl_, &error);
        Error::handle(&error);
        return enable;
    }

    /**
     * @brief Processes a frame synchronously.
     *
     * @param frame The frame to be processed.
     * @return std::shared_ptr< Frame > The processed frame.
     */
    virtual std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame) {
        ob_error *error  = nullptr;
        auto      result = ob_filter_process(impl_, frame->getImpl(), &error);
        Error::handle(&error);
        return std::make_shared<Frame>(result);
    }

    /**
     * @brief Pushes the pending frame into the cache for asynchronous processing.
     *
     * @param frame The pending frame. The processing result is returned by the callback function.
     */
    virtual void pushFrame(std::shared_ptr<Frame> frame) {
        ob_error *error = nullptr;
        ob_filter_push_frame(impl_, frame->getImpl(), &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the callback function for asynchronous processing.
     *
     * @param callback The processing result callback.
     */
    virtual void setCallBack(FilterCallback callback) {
        callback_       = callback;
        ob_error *error = nullptr;
        ob_filter_set_callback(impl_, &Filter::filterCallback, this, &error);
        Error::handle(&error);
    }

private:
    static void filterCallback(ob_frame *frame, void *userData) {
        auto filter = static_cast<Filter *>(userData);
        filter->callback_(std::make_shared<Frame>(frame));
    }
};

/**
 *  @brief A factory class for creating filters.
 */
class FilterFactory {
public:
    /**
     * @brief Create a filter by name.
     */
    static std::shared_ptr<Filter> createFilter(const std::string &name) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter(name.c_str(), &error);
        Error::handle(&error);
        return std::make_shared<Filter>(impl);
    }

    /**
     * @brief Create a private filter by name and activation key.
     * @brief Some private filters require an activation key to be activated, its depends on the vendor of the filter.
     *
     * @param name The name of the filter.
     * @param activation_key The activation key of the filter.
     *
     */
    static std::shared_ptr<Filter> createPrivateFilter(const std::string &name, const std::string &activationKey) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter(name.c_str(), activationKey.c_str(), &error);
        Error::handle(&error);
        return std::make_shared<Filter>(impl);
    }

    /**
     * @brief Get the vendor specific code of a filter by filter name.
     * @brief A private filter can define its own vendor specific code for specific purposes.
     *
     * @param name The name of the filter.
     * @return std::string The vendor specific code of the filter.
     */
    static std::string getFilterVendorSpecificCode(const std::string &name) {
        ob_error *error = nullptr;
        auto      code  = ob_filter_get_vendor_specific_code(name.c_str(), &error);
        Error::handle(&error);
        return code;
    }
};

}  // namespace ob
