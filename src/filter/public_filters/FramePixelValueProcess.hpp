#pragma once
#include "FilterBase.hpp"

namespace libobsensor {

class PixelValueScaler : public FilterBase {
public:
    PixelValueScaler(const std::string &name);
    virtual ~PixelValueScaler() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

protected:
    float scale_ = 1.0f;
};

class PixelValueCutOff : public FilterBase {
public:
    PixelValueCutOff(const std::string &name);
    virtual ~PixelValueCutOff() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

protected:
    uint32_t min_ = 0;
    uint32_t max_ = 16000;
};

}  // namespace libobsensor
