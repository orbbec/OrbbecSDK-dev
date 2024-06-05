#include "DisparityProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "openobsdk/h/ObTypes.h"

namespace libobsensor {

DisparityTransform::DisparityTransform(const std::string &name) : FilterBase(name) {}

DisparityTransform::~DisparityTransform() noexcept {
    if(lookUpTable_ != nullptr) {
        delete[] lookUpTable_;
    }
}

void DisparityTransform::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 0) {
        throw unsupported_operation_exception("DisparityTransform update config error: unsupported operation.");
    }
}

const std::string &DisparityTransform::getConfigSchema() const {
    throw unsupported_operation_exception("DisparityTransform get config schema error: unsupported operation.");
}

void DisparityTransform::updateParam() {
    // disparityParam_ = param.disparityParam;
    // maxDepthValue_  = param.maxDepthValue;
    // minDepthValue_  = param.minDepthValue;
    // shiftScale_     = 1.0f / param.depthUnit;
    // for(int i = 8; i <= 16; i++) {
    //     uint16_t maxValue = (uint16_t)1 << i;
    //     if(maxValue == 0 || maxValue > shiftScale_ * maxDepthValue_) {
    //         outputPixelAvailableBitSize_ = i;
    //         break;
    //     }
    // }

    // lookUpTableSize_ = pow(2, disparityParam_.bitSize);  // 2^bitSize
    // if(lookUpTable_ != nullptr) {
    //     delete[] lookUpTable_;
    // }
    // lookUpTable_ = new uint16_t[lookUpTableSize_];
    // memset(lookUpTable_, 0, lookUpTableSize_ * sizeof(uint16_t));

    // if(param.isDualCamera) {
    //     dualCameraInitLookUpTable();
    // }
    // else {
    //     singleCameraInitLookUpTable();
    // }
}

void DisparityTransform::dualCameraInitLookUpTable() {
    // double   fb           = disparityParam_.baseline * disparityParam_.fx;
    // double   disparity    = 0;
    // double   depth        = 0;
    // uint32_t maxDisparity = 255;

    // for(int packedValue = 0; packedValue < lookUpTableSize_; packedValue++) {
    //     // unpack disparity
    //     if(disparityParam_.packMode == OB_DISP_PACK_ORIGINAL_NEW) {
    //         disparity = (double)packedValue / pow(2, disparityParam_.bitSize - 8);
    //     }
    //     else if(disparityParam_.packMode == OB_DISP_PACK_GEMINI2XL) {
    //         disparity    = (double)packedValue / pow(2, disparityParam_.bitSize - 10);
    //         maxDisparity = 1024;
    //     }
    //     else {
    //         throw libobsensor::unsupported_operation_exception("Unsupported disparity data pack type!");
    //     }

    //     if(fabs(disparity - disparityParam_.invalidDisp) < 0.0001) {
    //         lookUpTable_[packedValue] = 0;
    //         continue;
    //     }

    //     if(disparity < maxDisparity && disparity > 0) {
    //         disparity = disparity + disparityParam_.dispOffset;
    //         // disparity to depth
    //         // z = fx*baseline/disparity
    //         // 乘disparityParam_.unit:后将单位装换为1mm
    //         // 乘shiftScale_:将单位转换为用户设定的单位
    //         depth = disparityParam_.unit * shiftScale_ * (fb / disparity);
    //         if(depth <= maxDepthValue_ * shiftScale_ && depth >= minDepthValue_ * shiftScale_ && depth < 65536) {
    //             lookUpTable_[packedValue] = depth;
    //         }
    //     }
    // }
}

void DisparityTransform::singleCameraInitLookUpTable() {
    // double disparity = 0;
    // double depth     = 0;

    // for(uint16_t packedValue = 0; packedValue < lookUpTableSize_; packedValue++) {
    //     // unpack disparity
    //     if(disparityParam_.packMode == OB_DISP_PACK_ORIGINAL_NEW) {
    //         // todo: 当前参数是bitSize=14的
    //         if(packedValue == 8192) {
    //             continue;
    //         }
    //         disparity = ((packedValue + 8192) & 0x3FFF) - 8192;  // 左
    //         disparity /= pow(2, disparityParam_.bitSize - 8);  // 传输打包时，将视差左移了，保留了更多亚像素精度（小数部分精度），所以这里要除64
    //     }
    //     else {
    //         throw libobsensor::unsupported_operation_exception("Unsupported disparity data pack type!");
    //     }

    //     disparity = disparity + disparityParam_.dispOffset;

    //     // disparity to depth
    //     // zpps = zpd/fx
    //     // z = zpd/(1-disparity*zpd/fx/baseline)  = zpd/(1-disparity*zpps/baseline)
    //     // 但是，由针孔模型可知，视差方向与真实相反，视差值需要*-1
    //     // 所以最后得到公式： z = zpd/(1+disparity*zpps/baseline)
    //     //
    //     // 乘disparityParam_.unit:后将单位转换为1mm
    //     // 乘shiftScale_:将单位转换为用户设定的单位
    //     depth = (double)disparityParam_.unit * shiftScale_ * disparityParam_.zpd / (1 + disparity * disparityParam_.zpps / disparityParam_.baseline);
    //     // depth = (double)shiftScale_ * disparityParam_.zpd / (1 + disparity * disparityParam_.zpps / disparityParam_.baseline);
    //     // check cut-offs
    //     if((depth >= minDepthValue_ * shiftScale_) && (depth <= maxDepthValue_ * shiftScale_) && depth < 65536) {
    //         lookUpTable_[packedValue] = (uint16_t)depth;
    //     }
    // }
}

void DisparityTransform::checkDisparityConverterParams(std::shared_ptr<Frame> frame) {
    // bool  scaleValueChanged = false;
    // float scaleValue        = frame->as<DepthFrame>()->getValueScale();

    // auto streamProfile = frame->getStreamProfile();

    // if(scaleValue != depthProcessorParam_.depthUnit) {
    //     depthProcessorParam_.depthUnit = scaleValue;
    //     scaleValueChanged              = true;
    // }

    // bool streamProfileChanged = false;
    // auto streamProfile        = frame->getStreamProfile();
    // if(!currentStreamProfile_ || currentStreamProfile_ != streamProfile) {
    //     currentStreamProfile_                         = streamProfile;
    //     auto                    algPmManager          = streamProfile->getAlgParamManager();
    //     auto                    strLightParamManager  = std::dynamic_pointer_cast<StructuredLightAlgParamManager>(algPmManager);
    //     OBDisparityProcessParam disparityProcessParam = strLightParamManager->getDisparityProcessParam(streamProfile);

    //     depthProcessorParam_.isDualCamera   = strLightParamManager->isBinocularCamera();
    //     depthProcessorParam_.disparityParam = disparityProcessParam;
    //     streamProfileChanged                = true;
    // }

    // if(scaleValueChanged || streamProfileChanged) {
    //     disparity2DepthConverter_          = std::make_shared<Disparity2DepthConverter>();
    //     depthProcessorParam_.maxDepthValue = 65535;
    //     depthProcessorParam_.minDepthValue = 0;
    //     disparity2DepthConverter_->updateParam(depthProcessorParam_);
    // }
}

std::shared_ptr<Frame> DisparityTransform::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    auto outFrame = FrameFactory::cloneFrame(frame);
    std::shared_ptr<VideoFrame> depthFrame = nullptr;
    if(frame->is<FrameSet>()) {
        depthFrame = outFrame->as<FrameSet>()->getDepthFrame()->as<VideoFrame>();
    }
    else {
        depthFrame = outFrame->as<VideoFrame>();
    }

    OBFormat frameFormat = depthFrame->getFormat();
    if(frameFormat != OB_FORMAT_DISP16) {
        return outFrame;
    }

    // Check disparity converter params.
    // checkDisparityConverterParams(depthFrame);

    // int       frameSize = depthFrame->getDataSize() / 2;
    // uint16_t *pPixel    = (uint16_t *)depthFrame->getData();

    // // 视差数据打包后，还会根据实际需要右移n位牺牲精度以降低带宽，所以需要先左移回来
    // int8_t   offset = disparityParam_.bitSize - depthFrame->getPixelAvailableBitSize();
    // uint16_t mask   = (uint16_t)0xffff >> (16 - disparityParam_.bitSize);
    // if(offset >= 0) {
    //     while(frameSize--) {
    //         *pPixel = lookUpTable_[((*pPixel) << offset) & mask];
    //         pPixel++;
    //     }
    // }
    // else {
    //     offset = -offset;
    //     while(frameSize--) {
    //         *pPixel = lookUpTable_[((*pPixel) >> offset) & mask];
    //         pPixel++;
    //     }
    // }

    // depthFrame->setPixelAvailableBitSize(outputPixelAvailableBitSize_);
    // depthFrame->as<DepthFrame>()->setValueScale(1.0f / shiftScale_);
    //depthFrame->as<DepthFrame>()->setFormat(OB_FORMAT_Y16);

    /*if(outFrame->is<FrameSet>()) {
        auto frameSet = outFrame->as<FrameSet>();
        frameSet->pushFrame(OB_FRAME_DEPTH, std::move(newFrame));
        newFrame = frameSet;
    }*/

    return outFrame;
}

}  // namespace libobsensor