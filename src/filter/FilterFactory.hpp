#pragma once
#include "IFilter.hpp"
#include "private_filters/PrivFilterLoader.hpp"
#include <memory>

namespace libobsensor {

class FilterFactory {
public:
    static std::shared_ptr<FilterFactory> getInstance();
    static void                           destroyInstance();

    ~FilterFactory() noexcept;

    std::shared_ptr<IFilter>        createFilter(const std::string &filterName);
    std::shared_ptr<IFilter>        createPrivateFilter(const std::string &filterName, const std::string &activationKey);
    std::shared_ptr<IFilterCreator> getFilterCreator(const std::string &filterName);

private:
    FilterFactory();

private:
    std::map<std::string, std::shared_ptr<IFilterCreator>> filterCreators_;
};

}  // namespace libobsensor