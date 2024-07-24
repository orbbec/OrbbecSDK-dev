#include "FrameProcessor.hpp"
#include "frame/FrameFactory.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {
FrameProcessorFactory::FrameProcessorFactory(IDevice *owner) : DeviceComponentBase(owner) {
    dylib_ = std::make_shared<dylib>(moduleLoadPath_.c_str(), "ob_frame_processor");

    auto dylib = dylib_;
    context_   = std::shared_ptr<FrameProcessorContext>(new FrameProcessorContext(), [dylib](FrameProcessorContext *context) {
        if(context && context->destroy_context) {
            ob_error *error = nullptr;
            context->destroy_context(context->context, &error);
            delete error;
        }
    });

    if(dylib_) {
        context_->create_context = dylib_->get_function<ob_frame_processor_context *(ob_device *, ob_error **)>("ob_create_frame_processor_context");
        context_->create_processor =
            dylib_->get_function<ob_frame_processor *(ob_frame_processor_context *, ob_sensor_type type, ob_error **)>("ob_create_frame_processor");
        context_->get_config_schema = dylib_->get_function<const char *(ob_frame_processor *, ob_error **)>("ob_frame_processor_get_config_schema");
        context_->update_config     = dylib_->get_function<void(ob_frame_processor *, size_t, const char **, ob_error **)>("ob_frame_processor_update_config");
        context_->process_frame     = dylib_->get_function<ob_frame *(ob_frame_processor *, ob_frame *, ob_error **)>("ob_frame_processor_process_frame");
        context_->destroy_processor = dylib_->get_function<void(ob_frame_processor *, ob_error **)>("ob_destroy_frame_processor");
        context_->destroy_context   = dylib_->get_function<void(ob_frame_processor_context *, ob_error **)>("ob_destroy_frame_processor_context");
    }

    if(context_->create_context && !context_->context) {
        ob_device cDevice;
        cDevice.device    = owner->shared_from_this();
        ob_error *error   = nullptr;
        context_->context = context_->create_context(&cDevice, &error);
        if(error) {
            // TODO
            throw std::runtime_error("create frame processor context failed");
        }
    }
}

FrameProcessorFactory::~FrameProcessorFactory() noexcept = default;

std::shared_ptr<FrameProcessor> FrameProcessorFactory::createFrameProcessor(OBSensorType sensorType) {
    if(context_ && (!context_->context || context_->create_processor == nullptr)) {
        return nullptr;
    }

    auto iter = frameProcessors_.find(sensorType);
    if(iter != frameProcessors_.end()) {
        return iter->second;
    }

    auto                            owner = getOwner();
    std::shared_ptr<FrameProcessor> processor;
    TRY_EXECUTE({ processor = std::make_shared<FrameProcessor>(owner, context_, sensorType); })
    frameProcessors_.insert({ sensorType, processor });
    return processor;
}

FrameProcessor::FrameProcessor(IDevice *owner, std::shared_ptr<FrameProcessorContext> context, OBSensorType sensorType)
    : FilterBase("FrameProcessor"), DeviceComponentBase(owner), context_(context), sensorType_(sensorType) {
    if(context_->context && context_->create_processor) {
        ob_error *error   = nullptr;
        privateProcessor_ = context_->create_processor(context_->context, sensorType, &error);
        if(error) {
            auto msg = std::string(error->message);
            delete error;
            throw std::runtime_error(msg);
        }
    }

    if(context_->context && context_->get_config_schema) {
        ob_error   *error  = nullptr;
        const char *schema = context_->get_config_schema(privateProcessor_, &error);
        if(error) {
            delete error;
        }
        if(schema) {
            configSchema_ = std::string(schema);
        }
    }
}

FrameProcessor::~FrameProcessor() noexcept {
    if(context_->destroy_processor) {
        ob_error *error = nullptr;
        context_->destroy_processor(privateProcessor_, &error);
        if(error) {
            delete error;
        }
    }
}

std::shared_ptr<Frame> FrameProcessor::processFunc(std::shared_ptr<const Frame> frame) {
    if(!context_->process_frame || !privateProcessor_) {
        return FrameFactory::createFrameFromOtherFrame(frame, true);
    }
    ob_frame              *c_frame = new ob_frame();
    ob_error              *error   = nullptr;
    std::shared_ptr<Frame> resultFrame;
    c_frame->frame = std::const_pointer_cast<Frame>(frame);

    auto rst_frame = context_->process_frame(privateProcessor_, c_frame, &error);
    if(rst_frame) {
        resultFrame = rst_frame->frame;
        delete rst_frame;
    }
    delete c_frame;

    if(error) {
        delete error;
        return FrameFactory::createFrameFromOtherFrame(frame, true);
    }

    return resultFrame;
}

const std::string &FrameProcessor::getConfigSchema() const {
    return configSchema_;
}

void FrameProcessor::updateConfig(std::vector<std::string> &params) {
    ob_error                 *error = nullptr;
    std::vector<const char *> c_params;
    for(auto &p: params) {
        c_params.push_back(p.c_str());
    }
    context_->update_config(privateProcessor_, params.size(), c_params.data(), &error);
    if(error) {
        delete error;
    }
}

void FrameProcessor::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    switch(propertyId) {
    case OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL: {
        setConfigValue("DisparityTransform#255", static_cast<double>(value.intValue));
    } break;
    case OB_PROP_DEPTH_PRECISION_LEVEL_INT: {
        auto level     = static_cast<OBDepthPrecisionLevel>(value.intValue);
        auto depthUnit = utils::depthPrecisionLevelToUnit(level);
        setConfigValue("DisparityTransform#2", static_cast<double>(depthUnit));
    }
    case OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT: {
        setConfigValue("DisparityTransform#2", static_cast<double>(value.floatValue));
    } break;
    default:
        throw invalid_value_exception("Invalid property id");
    }
}

void FrameProcessor::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    switch(propertyId) {
    case OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL: {
        auto getValue   = getConfigValue("DisparityTransform#255");
        value->intValue = static_cast<int32_t>(getValue);
    } break;
    case OB_PROP_DEPTH_PRECISION_LEVEL_INT: {
        auto getValue       = getConfigValue("DisparityTransform#2");
        auto precisionLevel = utils::depthUnitToPrecisionLevel(static_cast<float>(getValue));
        value->intValue     = static_cast<int32_t>(precisionLevel);
    }
    case OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT: {
        auto getValue     = getConfigValue("DisparityTransform#2");
        value->floatValue = static_cast<float>(getValue);
    } break;
    default:
        throw invalid_value_exception("Invalid property id");
    }
}

void FrameProcessor::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    double value = 0.0f;
    switch(propertyId) {
    case OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL: {
        value                = getConfigValue("DisparityTransform#255");
        range->cur.intValue  = static_cast<int32_t>(value);
        range->def.intValue  = 1;
        range->max.intValue  = 1;
        range->min.intValue  = 0;
        range->step.intValue = 1;
    } break;
    case OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT: {
        value                  = getConfigValue("DisparityTransform#2");
        range->cur.floatValue  = static_cast<float>(value);
        range->def.floatValue  = 1.0f;
        range->max.floatValue  = 10.0f;
        range->min.floatValue  = 0.001f;
        range->step.floatValue = 0.001f;
    } break;
    case OB_PROP_DEPTH_PRECISION_LEVEL_INT: {
        OBPropertyValue cur;
        getPropertyValue(OB_PROP_DEPTH_PRECISION_LEVEL_INT, &cur);
        range->cur.intValue  = cur.intValue;
        range->def.intValue  = static_cast<int32_t>(OB_PRECISION_1MM);
        range->max.intValue  = static_cast<int32_t>(OB_PRECISION_1MM);
        range->min.intValue  = static_cast<int32_t>(OB_PRECISION_0MM05);
        range->step.intValue = 1;
    } break;
    default:
        throw invalid_value_exception("Invalid property id");
    }
}
}  // namespace libobsensor