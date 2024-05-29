#pragma once
#include <string>
#include <memory>
#include <map>
#include <dylib.hpp>
#include "IFilter.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include "PrivFilterTypes.h"
#ifdef __cplusplus
}
#endif

namespace libobsensor {

struct PrivFilterLibHandle {
    std::string                                   dir;
    std::string                                   libName;
    std::shared_ptr<dylib>                        dylib;
    pfunc_ob_get_filter_count                     get_filter_count;
    pfunc_ob_get_filter_name                      get_filter_name;
    pfunc_ob_create_filter                        create_filter;
    pfunc_ob_priv_filter_get_vendor_specific_code get_vendor_specific_code;
    pfunc_ob_priv_filter_is_activated             is_activated;
    pfunc_ob_priv_filter_activate                 activate;
};

class PrivFilterCreator : public IPrivFilterCreator {
public:
    PrivFilterCreator(std::shared_ptr<PrivFilterLibHandle> libHandle, size_t index);
    virtual ~PrivFilterCreator() = default;

    std::shared_ptr<IFilter> create() override;
    std::shared_ptr<IFilter> create(const std::string &activationKey) override;
    std::string              getVendorSpecificCode() const override;

private:
    std::shared_ptr<PrivFilterLibHandle> libHandle_;
    size_t                               index_;
};

namespace PrivFilterCreatorLoader {
std::map<std::string, std::shared_ptr<IFilterCreator>> getCreators();
}

}  // namespace libobsensor