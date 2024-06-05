#pragma once
#include "FilterBase.hpp"
#include <mutex>

namespace libobsensor {

class DecimationFilter : public FilterBase {
public:
    DecimationFilter(const std::string &name);
    virtual ~DecimationFilter() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    bool isFrameFormatTypeSupported(OBFormat type);
    void updateOutputProfile(const std::shared_ptr<Frame> frame);
    void decimateDepth(uint16_t *frame_data_in, uint16_t *frame_data_out, size_t width_in, size_t scale);
    void decimateOthers(OBFormat format, void *frame_data_in, void *frame_data_out, size_t width_in, size_t scale);

protected:
    std::recursive_mutex                                                                           scaleChangedMutex_;
    std::map<std::tuple<const VideoStreamProfile *, uint8_t>, std::shared_ptr<VideoStreamProfile>> registered_profiles_;
    std::shared_ptr<const VideoStreamProfile>                                                      source_stream_profile_;
    std::shared_ptr<VideoStreamProfile>                                                            target_stream_profile_;

    uint8_t  decimation_factor_;
    uint8_t  control_val_;
    uint8_t  patch_size_;
    uint8_t  kernel_size_;
    uint16_t real_width_;    // Number of rows/columns with real datain the decimated image
    uint16_t real_height_;   // Correspond to w,h in the reference code
    uint16_t padded_width_;  // Corresponds to w4/h4 in the reference code
    uint16_t padded_height_;
    bool     recalc_profile_;
    bool     options_changed_;  // Tracking changes imposed by user
};

}  // namespace libobsensor
