#pragma once
#include "IDevice.hpp"
#include "filter/FilterBase.hpp"
#include "PrivFrameProcessorTypes.h"
#include <dylib.hpp>
#include <map>

namespace libobsensor {
class FrameProcessor;
struct FrameProcessorContext {
    ob_frame_processor_context *context = nullptr;

    pfunc_ob_create_frame_processor_context    create_context    = nullptr;
    pfunc_ob_create_frame_processor            create_processor  = nullptr;
    pfunc_ob_frame_processor_get_config_schema get_config_schema = nullptr;
    pfunc_ob_frame_processor_update_config     update_config     = nullptr;
    pfunc_ob_frame_processor_process_frame     process_frame     = nullptr;
    pfunc_ob_destroy_frame_processor           destroy_processor = nullptr;
    pfunc_ob_destroy_frame_processor_context   destroy_context   = nullptr;
};

class FrameProcessorFactory final {
public:
    explicit FrameProcessorFactory(IDevice *device);
    ~FrameProcessorFactory() noexcept;

    std::shared_ptr<FrameProcessor> createFrameProcessor(OBSensorType sensorType);

private:
    std::string moduleLoadPath_ = "./frameprocessor/";

    std::shared_ptr<dylib> dylib_;

    std::shared_ptr<FrameProcessorContext> context_;

    std::map<OBSensorType,std::shared_ptr<FrameProcessor>> frameProcessors_;
};

class FrameProcessor : public FilterBase,public IPropertyPort {
public:
    FrameProcessor(std::shared_ptr<FrameProcessorContext> context, OBSensorType sensorType);

    ~FrameProcessor() noexcept;

    const std::string &getConfigSchema() const override;

    void updateConfig(std::vector<std::string> &params);

    OBSensorType getSensorType() {
        return sensorType_;
    }

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;

    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

protected:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame);

private:
    std::shared_ptr<FrameProcessorContext> context_;

    OBSensorType sensorType_;

    std::string configSchema_;

    ob_frame_processor *privateProcessor_;
};

}  // namespace libobsensor