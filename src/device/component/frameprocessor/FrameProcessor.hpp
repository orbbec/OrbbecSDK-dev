#pragma once
#include "interface/IDevice.hpp"
#include "PrivFrameProcessorTypes.h"
#include <dylib.hpp>
#include <map>

namespace libobsensor{
class FrameProcessor;
struct FrameProcessorContext
{
    ob_frame_processor_context *context = nullptr;
    
    pfunc_ob_create_frame_processor_context create_context = nullptr;
    pfunc_ob_create_frame_processor create_processor = nullptr;
    pfunc_ob_frame_processor_get_config_schema get_config_schema = nullptr;
    pfunc_ob_frame_processor_update_config update_config = nullptr;
    pfunc_ob_frame_processor_process_frame process_frame = nullptr;
    pfunc_ob_destroy_frame_processor destroy_processor = nullptr;
    pfunc_ob_destroy_frame_processor_context destroy_context = nullptr;
};

class FrameProcessorFactory final{
public:
    explicit FrameProcessorFactory(std::shared_ptr<IDevice> device);
    ~FrameProcessorFactory() noexcept;

    std::shared_ptr<FrameProcessor> createFrameProcessor(OBSensorType sensorType);
private:
    const std::string &moduleLoadPath_ = "./frameprocessor/";

    std::shared_ptr<dylib> dylib_;

    FrameProcessorContext context_;
};

class FrameProcessor final{
public:
    FrameProcessor(FrameProcessorContext *context,OBSensorType sensorType);

    ~FrameProcessor() noexcept;

    std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame);

    const std::string &getConfigSchema();

    void updateConfig(std::vector<std::string> &params);

private:    
    FrameProcessorContext *context_;

    OBSensorType sensorType_;

    std::string configSchema_;

    ob_frame_processor *privateProcessor_;
};

}//namespace libobsensor