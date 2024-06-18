#include "FrameProcessor.hpp"
#include "core/frame/FrameFactory.hpp"
#include "shared/utils/Utils.hpp"

namespace libobsensor {
FrameProcessorFactory::FrameProcessorFactory(std::shared_ptr<IDevice> device){
    //TODO
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
            utils::safeDelete(error);
        }
    }
}

std::shared_ptr<FrameProcessor> FrameProcessorFactory::createFrameProcessor(OBSensorType sensorType){
    return std::make_shared<FrameProcessor>(&context_,sensorType);
}


FrameProcessor::FrameProcessor(FrameProcessorContext *context,OBSensorType sensorType) : context_(context),sensorType_(sensorType){
    if(context_->context && context_->create_processor){
        ob_error *error = nullptr;
        privateProcessor_ = context_->create_processor(context_->context,sensorType,&error);
        if(error){
            //TODO
            throw std::runtime_error("create frame processor failed");
        }

        const char *schema = context_->get_config_schema(privateProcessor_, &error);
        if(error){
            //TODO
            utils::safeDelete(error);
        }
        if(schema){
            configSchema_ = std::string(schema);
        }
    }
}

FrameProcessor::~FrameProcessor() noexcept{
    if(context_->destroy_processor){
        ob_error *error = nullptr;
        context_->destroy_processor(privateProcessor_, &error);
        if(error){
            //TODO
            utils::safeDelete(error);
        }
    }
}

std::shared_ptr<Frame> FrameProcessor::process(std::shared_ptr<const Frame> frame){
    if(context_->process_frame){
        ob_frame *c_frame = new ob_frame();
        ob_error *error = nullptr;
        std::shared_ptr<Frame> resultFrame;
        c_frame->frame    = std::const_pointer_cast<Frame>(frame);
        
        auto rst_frame = context_->process_frame(privateProcessor_,c_frame,&error);
        if(rst_frame) {
            resultFrame = rst_frame->frame;
            delete rst_frame;
        }
        delete c_frame;
        
        if(error){
            //TODO
            utils::safeDelete(error);
            return FrameFactory::cloneFrame(frame,true);
        }
        return resultFrame;
    }

    return FrameFactory::cloneFrame(frame,true);
}

const std::string &FrameProcessor::getConfigSchema(){
    return configSchema_;
}

void FrameProcessor::updateConfig(std::vector<std::string> &params){
    ob_error *error = nullptr;
    std::vector<const char *> c_params;
    for(auto &p: params) {
        c_params.push_back(p.c_str());
    }
    context_->update_config(privateProcessor_, params.size(), c_params.data(), &error);
    if(error){
        //TODO
        utils::safeDelete(error);
    }
}

}