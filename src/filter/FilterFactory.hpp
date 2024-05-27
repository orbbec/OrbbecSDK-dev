#pragma once
#include "IFilter.hpp"
#include <memory>

namespace libobsensor {

class FilterFactory {
public:
    static std::shared_ptr<FilterFactory> getInstance();
    static void                           destroyInstance();

    ~FilterFactory() noexcept;

    std::shared_ptr<IFilter> createPublicFilter(const std::string &filterName);
    std::shared_ptr<IFilter> createPrivateFilter(const std::string &filterName, const std::string &activationKey);

private:
    FilterFactory();
};

}  // namespace libobsensor