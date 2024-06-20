#include "HdrMergeProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "openobsdk/h/ObTypes.h"
#include "utils/PublicTypeHelper.hpp"

namespace libobsensor {

// avoids returning too old merged frame - frame counter jumps forward
const int SEQUENTIAL_FRAMES_THRESHOLD = 4;
// for infrared 8 bit per pixel
const int IR_UNDER_SATURATED_VALUE_Y8 = 0x05;  // 5
const int IR_OVER_SATURATED_VALUE_Y8  = 0xfa;  // 250 (255 - IR_UNDER_SATURATED_VALUE_Y8)
// for infrared 10 bit per pixel
const int IR_UNDER_SATURATED_VALUE_Y16 = 0x14;   // 20 (4 * IR_UNDER_SATURATED_VALUE_Y8)
const int IR_OVER_SATURATED_VALUE_Y16  = 0x3eb;  // 1003 (1023 - IR_UNDER_SATURATED_VALUE_Y16)


template <typename T> bool isInfraredValid(T ir_value, OBFormat ir_format) {
    bool result = false;
    if(ir_format == OB_FORMAT_Y8)
        result = (ir_value > IR_UNDER_SATURATED_VALUE_Y8) && (ir_value < IR_OVER_SATURATED_VALUE_Y8);
    else if(ir_format == OB_FORMAT_Y16)
        result = (ir_value > IR_UNDER_SATURATED_VALUE_Y16) && (ir_value < IR_OVER_SATURATED_VALUE_Y16);
    else
        result = false;
    return result;
}

template <typename T>
void mergeFramesUsingIr(uint16_t *new_data, uint16_t *d0, uint16_t *d1, const std::shared_ptr<const Frame> first_ir, const std::shared_ptr<const Frame> second_ir,
                        int width_height_prod) {
    auto i0 = (T *)first_ir->getData();
    auto i1 = (T *)second_ir->getData();

    auto format = first_ir->getFormat();
    for(int i = 0; i < width_height_prod; i++) {
        if(isInfraredValid<T>(i0[i], format) && d0[i])
            new_data[i] = d0[i];
        else if(isInfraredValid<T>(i1[i], format) && d1[i])
            new_data[i] = d1[i];
        else
            new_data[i] = 0;
    }
}

void mergeFramesUsingOnlyDepth(uint16_t *new_data, uint16_t *d0, uint16_t *d1, int width_height_prod) {
    for(int i = 0; i < width_height_prod; i++) {
        if(d0[i])
            new_data[i] = d0[i];
        else if(d1[i])
            new_data[i] = d1[i];
        else
            new_data[i] = 0;
    }
}

std::shared_ptr<const IRFrame> getIRFrameFromFrameSet(std::shared_ptr<const Frame> frame_fs) {
    if(!frame_fs->is<FrameSet>()) {
        return nullptr;
    }

    auto frameSet = frame_fs->as<FrameSet>();
    auto irFrame  = frameSet->getFrame(OB_FRAME_IR);
    if(!irFrame) {
        irFrame = frameSet->getFrame(OB_FRAME_IR_LEFT);
        if(!irFrame) {
            irFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
        }
    }

    if(irFrame) {
        return irFrame->as<IRFrame>();
    }

    return nullptr;
}

bool shouldIrBeUsedForMerging(std::shared_ptr<const DepthFrame> first_depth, std::shared_ptr<const IRFrame> first_ir,
                              std::shared_ptr<const DepthFrame> second_depth, std::shared_ptr<const IRFrame> second_ir) {

    // checking ir frames are not null
    bool use_ir = (first_ir && second_ir);

    if(use_ir) {
        // IR and Depth dimensions must be aligned
        if((first_depth->getHeight() != first_ir->getHeight()) || (first_depth->getWidth() != first_ir->getWidth())
           || (second_ir->getHeight() != first_ir->getHeight()) || (second_ir->getWidth() != first_ir->getWidth()))
            use_ir = false;
    }

    // checking frame counter of first depth and ir are the same
    if(use_ir) {
        // on devices that do not support meta data on IR frames, do not use IR for hdr merging
        int depth_frame_counter = static_cast<int>(first_depth->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER));
        int ir_frame_counter    = static_cast<int>(first_ir->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER));
        use_ir                  = (depth_frame_counter == ir_frame_counter);

        // checking frame counter of second depth and ir are the same
        if(use_ir) {
            depth_frame_counter = static_cast<int>(second_depth->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER));
            ir_frame_counter    = static_cast<int>(second_ir->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER));
            use_ir              = (depth_frame_counter == ir_frame_counter);

            // checking sequence id of first depth and ir are the same
            if(use_ir) {
                auto depth_seq_id = first_depth->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX);
                auto ir_seq_id    = first_ir->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX);
                use_ir            = (depth_seq_id == ir_seq_id);

                // checking sequence id of second depth and ir are the same
                if(use_ir) {
                    depth_seq_id = second_depth->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX);
                    ir_seq_id    = second_ir->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX);
                    use_ir       = (depth_seq_id == ir_seq_id);

                    // checking both ir have the same format
                    if(use_ir) {
                        use_ir = (first_ir->getFormat() == second_ir->getFormat());
                    }
                }
            }
        }
    }

    return use_ir;
}

bool checkFramesMergeAbility(std::shared_ptr<const Frame> first_f, std::shared_ptr<const Frame> second_f, bool &use_ir) {
    std::shared_ptr<const DepthFrame> first_depth  = nullptr;
    std::shared_ptr<const DepthFrame> second_depth = nullptr;

    std::shared_ptr<const IRFrame> first_ir     = nullptr;
    std::shared_ptr<const IRFrame> second_ir = nullptr;

    if(first_f->is<FrameSet>()) {
        first_depth = first_f->as<FrameSet>()->getFrame(OB_FRAME_DEPTH)->as<DepthFrame>();
    }
    else {
        first_depth = first_f->as<DepthFrame>();
    }
    first_ir = getIRFrameFromFrameSet(first_f);

    if(second_f->is<FrameSet>()) {
        second_depth = second_f->as<FrameSet>()->getFrame(OB_FRAME_DEPTH)->as<DepthFrame>();
    }
    else {
        second_depth = second_f->as<DepthFrame>();
    }
    second_ir = getIRFrameFromFrameSet(second_f);

    auto first_fs_frame_counter  = first_depth->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER);
    auto second_fs_frame_counter = second_depth->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER);

    // The aim of this checking is that the output merged frame will have frame counter n and
    // frame counter n and will be created by frames n and n+1
    if(first_fs_frame_counter + 1 != second_fs_frame_counter) {
        return false;
    }

    // Depth dimensions must align
    if((first_depth->getHeight() != second_depth->getHeight()) || (first_depth->getWidth() != second_depth->getWidth())) {
        return false;
    }

    use_ir = shouldIrBeUsedForMerging(first_depth, first_ir, second_depth, second_ir);

    return true;
}



HdrMerge::HdrMerge(const std::string &name) : FilterBase(name) {}
HdrMerge::~HdrMerge() noexcept {}

void HdrMerge::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 0) {
        throw unsupported_operation_exception("HdrMerge update config error: unsupported operation.");
    }
}

const std::string &HdrMerge::getConfigSchema() const {
    throw unsupported_operation_exception("HdrMerge get config schema error: unsupported operation.");
}

std::shared_ptr<Frame> HdrMerge::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    // 1.get depth frame,check the depth frame exists
    std::shared_ptr<const DepthFrame> depthFrame = nullptr;
    std::shared_ptr<Frame>            outFrame   = FrameFactory::cloneFrame(frame);
    if(frame->is<FrameSet>()) {
        depthFrame = frame->as<FrameSet>()->getFrame(OB_FRAME_DEPTH)->as<DepthFrame>();
    }
    else {
        depthFrame = frame->as<DepthFrame>();
    }

    if(!depthFrame) {
        LOG_WARN_INTVL("No depth frame found, hdrMerge unsupported to process this frame");
        return outFrame;
    }

    auto depth_seq_size = depthFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_SIZE);
    if(depth_seq_size != 2) {
        return outFrame;
    }

    // 2.add the frame to vector of frames
    auto depth_seq_id = depthFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX);

    // condition added to ensure that frames are saved in the right order
    // to prevent for example the saving of frame with sequence id 1 before
    // saving frame of sequence id 0
    // so that the merging with be deterministic - always done with frame n and n+1
    // with frame n as basis
    int64_t frameSize = static_cast<int64_t>(frames_.size());
    if(frameSize == depth_seq_id) {
        frames_[depth_seq_id] = outFrame;
    }

    // discard merged frame if not relevant
    discardDepthMergedFrameIfNeeded(outFrame);

    // 3. check if size of this vector is at least 2 (if not - return latest merge frame)
    if(frames_.size() >= 2) {
        // 4. pop out both framesets from the vector
        std::shared_ptr<const Frame> frame_0 = frames_[0];
        std::shared_ptr<const Frame> frame_1 = frames_[1];
        frames_.clear();

        bool use_ir = false;
        if(checkFramesMergeAbility(frame_0, frame_1, use_ir)) {
            // 5. apply merge algo
            std::shared_ptr<Frame> new_frame = mergingAlgorithm(frame_0, frame_1, use_ir);
            if(new_frame) {
                // 6. save merge frame as latest merge frame
                depth_merged_frame_ = new_frame;
            }
        }
    }

    // 7. return the merge frame
    if(depth_merged_frame_) {
        if(outFrame->is<FrameSet>()) {
            auto frameSet = outFrame->as<FrameSet>();
            frameSet->pushFrame(std::move(depth_merged_frame_));
            return outFrame;
        }
        else {
            return depth_merged_frame_;
        }
    }

    return outFrame;
}

void HdrMerge::discardDepthMergedFrameIfNeeded(std::shared_ptr<Frame> frame) {
    if(depth_merged_frame_) {
        // criteria for discarding saved merged_depth_frame:
        // 1 - frame counter for merged depth is greater than the input frame
        // 2 - resolution change
        std::shared_ptr<const DepthFrame> newDepthFrame = nullptr;
        if(frame->is<FrameSet>()) {
            newDepthFrame = frame->as<FrameSet>()->getFrame(OB_FRAME_DEPTH)->as<DepthFrame>();
        }
        else {
            newDepthFrame = frame->as<DepthFrame>();
        }

        auto mergedFrame = depth_merged_frame_->as<DepthFrame>();

        auto depth_merged_frame_counter           = depth_merged_frame_->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER);
        auto input_frame_counter                  = newDepthFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER);
        bool counter_diff_over_threshold_detected = ((input_frame_counter - depth_merged_frame_counter) >= SEQUENTIAL_FRAMES_THRESHOLD);
        bool restart_pipe_detected                = (depth_merged_frame_counter > input_frame_counter);

        bool resolution_change_detected = (mergedFrame->getWidth() != newDepthFrame->getWidth()) || (mergedFrame->getHeight() != newDepthFrame->getHeight());

        if(restart_pipe_detected || resolution_change_detected || counter_diff_over_threshold_detected) {
            depth_merged_frame_ = nullptr;
        }
    }
}

std::shared_ptr<Frame> HdrMerge::mergingAlgorithm(std::shared_ptr<const Frame> first_fs, std::shared_ptr<const Frame> second_fs, const bool use_ir) {
    std::shared_ptr<const DepthFrame> first_depth  = nullptr;
    std::shared_ptr<const DepthFrame> second_depth = nullptr;
    std::shared_ptr<const IRFrame>    first_ir     = nullptr;
    std::shared_ptr<const IRFrame>    second_ir    = nullptr;

    if(first_fs->is<FrameSet>()) {
        first_depth = first_fs->as<FrameSet>()->getFrame(OB_FRAME_DEPTH)->as<DepthFrame>();
    }
    else {
        first_depth = first_fs->as<DepthFrame>();
    }
    first_ir = getIRFrameFromFrameSet(first_fs);

    if(second_fs->is<FrameSet>()) {
        second_depth = second_fs->as<FrameSet>()->getFrame(OB_FRAME_DEPTH)->as<DepthFrame>();
    }
    else {
        second_depth = second_fs->as<DepthFrame>();
    }
    second_ir = getIRFrameFromFrameSet(second_fs);

    // new frame allocation
    auto width  = first_depth->getWidth();
    auto height = first_depth->getHeight();

    auto newFrame = FrameFactory::createFrameFromStreamProfile(first_depth->getStreamProfile());
    if(newFrame) {
        newFrame->copyInfo(first_depth);
        auto d0       = (uint16_t *)first_depth->getData();
        auto d1       = (uint16_t *)second_depth->getData();
        auto new_data = (uint16_t *)newFrame->getData();
        memset(new_data, 0, newFrame->getDataSize());
        int width_height_product = width * height;
        if(use_ir) {
            if(first_ir->getFormat() == OB_FORMAT_Y8) {
                mergeFramesUsingIr<uint8_t>(new_data, d0, d1, first_ir, second_ir, width_height_product);
            }
            else if(first_ir->getFormat() == OB_FORMAT_Y16) {
                mergeFramesUsingIr<uint16_t>(new_data, d0, d1, first_ir, second_ir, width_height_product);
            }
            else {
                mergeFramesUsingOnlyDepth(new_data, d0, d1, width_height_product);
            }
        }
        else {
            mergeFramesUsingOnlyDepth(new_data, d0, d1, width_height_product);
        }
        return newFrame;
    }

    return FrameFactory::cloneFrame(first_fs,true);
}

}  // namespace libobsensor
