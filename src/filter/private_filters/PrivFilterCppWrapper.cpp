#include "PrivFilterCppWrapper.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {
PrivFilterCppWrapper::PrivFilterCppWrapper(const char *filterName, ob_private_filter *privFilter) : FilterBase(filterName), privFilter_(privFilter) {}

PrivFilterCppWrapper::~PrivFilterCppWrapper() noexcept {
    if(privFilter_) {
        ob_error *error = nullptr;
        privFilter_->destroy(privFilter_, &error);
        if(error) {
            LOG_WARN("Private filter {} destroyed failed: {}", name_, error->message);
            delete error;
        }
        privFilter_ = nullptr;
    }
    LOG_DEBUG("Private filter {} destroyed", name_);
}

void PrivFilterCppWrapper::updateConfig(std::vector<std::string> &params) {
    ob_error                 *error = nullptr;
    std::vector<const char *> c_params;
    for(auto &p: params) {
        c_params.push_back(p.c_str());
    }
    privFilter_->update_config(privFilter_->filter, params.size(), c_params.data(), &error);
    if(error) {
        LOG_WARN("Private filter {} update config failed: {}", name_, error->message);
        delete error;
    }
};

std::string PrivFilterCppWrapper::getConfigSchema() const {
    ob_error   *error = nullptr;
    const char *desc  = privFilter_->get_config_schema(privFilter_->filter, &error);
    if(error) {
        LOG_WARN("Private filter {} get config schema failed: {}", name_, error->message);
        delete error;
        return "";
    }
    return desc;
};

void PrivFilterCppWrapper::reset() {
    ob_error *error = nullptr;
    privFilter_->reset(privFilter_->filter, &error);
    if(error) {
        LOG_WARN("Private filter {} reset failed: {}", name_, error->message);
        delete error;
    }
};

std::shared_ptr<Frame> PrivFilterCppWrapper::processFunc(std::shared_ptr<const Frame> frame) {
    ob_error *error = nullptr;
    ob_frame *c_frame = new ob_frame();
    c_frame->frame    = std::const_pointer_cast<Frame>(frame);

    auto rst_frame = privFilter_->process(privFilter_->filter, c_frame, &error);
    if(error) {
        LOG_WARN("Private filter {} process failed: {}", name_, error->message);
        delete error;
        return nullptr;
    }
    return rst_frame->frame;
}

}  // namespace libobsensor