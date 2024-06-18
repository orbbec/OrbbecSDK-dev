#include "FrameProcessor.hpp"

namespace libobsensor {
FrameProcessorFactory::FrameProcessorFactory(std::shared_ptr<IDevice> device){
    dylib_ = std::make_shared<dylib>("./frameprocessor/","ob_frame_processor");
    context_.create_context         = dylib_->get_function<ob_frame_processor_context*(ob_device *,ob_error **)>("ob_create_frame_processor_context");
    context_.create_processor       = dylib_->get_function<ob_frame_processor*(ob_frame_processor_context *,ob_sensor_type type,ob_error **)>("ob_create_frame_processor");
    context_.get_config_schema = dylib_->get_function<const char *(ob_frame_processor *,ob_error **)>("ob_frame_processor_get_config_schema");
    context_.update_config = dylib_->get_function<void(ob_frame_processor *,size_t, const char **,ob_error **)>("ob_frame_processor_update_config");
    context_.process_frame = dylib_->get_function<ob_frame*(ob_frame_processor *,ob_frame *,ob_error **)>("ob_frame_processor_process_frame");
    context_.destroy_processor = dylib_->get_function<void(ob_frame_processor *,ob_error **)>("ob_destroy_frame_processor");
    context_.destroy_context = dylib_->get_function<void(ob_frame_processor_context *,ob_error **)>("ob_destroy_frame_processor_context");

    if(context_.create_context && !context_.context){
        ob_device cDevice;
        cDevice.device = device;
        ob_error *error = nullptr;
        context_.context = context_.create_context(&cDevice,&error);
        if(error){
            //TODO
            throw std::runtime_error("create frame processor context failed");
        }
    }
}

FrameProcessorFactory::~FrameProcessorFactory() noexcept{
    if(context_.destroy_context){
        ob_error *error = nullptr;
        context_.destroy_context(context_.context, &error);
        if(error){
            //TODO
            ob_delete_error(error);
        }
    }

    if(context_.destroy_processor){
        ob_error *error = nullptr;
        for(const auto &iter : context_.processors){
            if(iter.second){
                context_.destroy_processor(iter.second, &error);
                if(error){
                    //TODO
                    ob_delete_error(error);
                }
            }
        }
    }
}

std::shared_ptr<FrameProcessor> FrameProcessorFactory::createFrameProcessor(OBSensorType sensorType){
    return std::make_shared<FrameProcessor>(&context_,sensorType);
}


FrameProcessor::FrameProcessor(FrameProcessorContext *context,OBSensorType sensorType) : context_(context),sensorType_(sensorType){
    if(context_->context && context_->create_processor){
        ob_error *error = nullptr;
        ob_frame_processor *processor = context_->create_processor(context_->context,sensorType,&error);
        if(error){
            //TODO
            throw std::runtime_error("create frame processor failed");
        }
        configSchema_ = context_->get_config_schema(processor, &error);
        if(error){
            //TODO
            ob_delete_error(error);
        }

        context_->processors.insert({sensorType,processor});
    }
}

std::shared_ptr<Frame> FrameProcessor::process(std::shared_ptr<const Frame> frame){
    auto processor = getFrameProcessor(sensorType_);
    if(processor){
        return nullptr;
    }

    if(context_->process_frame){
        ob_frame *c_frame = new ob_frame();
        ob_error *error = nullptr;
        c_frame->frame    = std::const_pointer_cast<Frame>(frame);
        auto rst_frame = context_->process_frame(processor,c_frame,&error);
        if(error){
            //TODO
            ob_delete_error(error);
            return nullptr;
        }
        return rst_frame->frame;
    }
    return nullptr;
}

const std::string &FrameProcessor::getConfigSchema(){
    return configSchema_;
}

void FrameProcessor::updateConfig(std::vector<std::string> &params){
    auto processor = getFrameProcessor(sensorType_);
    if(processor == nullptr){
        return;
    }
    ob_error *error = nullptr;
    std::vector<const char *> c_params;
    for(auto &p: params) {
        c_params.push_back(p.c_str());
    }
    context_->update_config(processor, params.size(), c_params.data(), &error);
    if(error){
        //TODO
        ob_delete_error(error);
    }
}

ob_frame_processor *FrameProcessor::getFrameProcessor(OBSensorType sensorType){
    if(context_->processors.find(sensorType) == context_->processors.end()){
        return nullptr;
    }
    return context_->processors[sensorType];
}

}