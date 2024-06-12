#pragma once
#include "FilterBase.hpp"
#include <mutex>

namespace libobsensor {

class DU08mmTo1mmConverter : public FilterBase {
public:
    DU08mmTo1mmConverter(const std::string &name);
    virtual ~DU08mmTo1mmConverter() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

};

}  // namespace libobsensor
