#include "SequenceIdProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "libobsensor/h/ObTypes.h"

namespace libobsensor {

SequenceIdFilter::SequenceIdFilter(const std::string &name) : FilterBase(name) {}
SequenceIdFilter::~SequenceIdFilter() noexcept {}

void SequenceIdFilter::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 1) {
        throw invalid_value_exception("SequenceIdFilter config error: params size not match");
    }
    try {

        uint32_t select_sequence_id = std::stoi(params[0]);
        if (select_sequence_id >= 0 && select_sequence_id <= 2) {
            if (select_sequence_id != selectedID_) {
                std::lock_guard<std::recursive_mutex> lk(valueUpdateMutex_);
                selectedID_ = select_sequence_id;
            }
        }
    }
    catch(const std::exception &e) {
        throw invalid_value_exception("SequenceIdFilter config error: " + std::string(e.what()));
    }
}

const std::string &SequenceIdFilter::getConfigSchema() const {
    // csv format: name，type， min，max，step，default，description
    static const std::string schema = "sequenceid, int, 0, 2, 1, 0, frame data sequence id value";
    return schema;
}

std::shared_ptr<Frame> SequenceIdFilter::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    auto outFrame = FrameFactory::createFrameFromOtherFrame(frame);
    if(outFrame->is<FrameSet>()) {
        LOG_WARN_INTVL("The Frame processed by SequenceIdFilter cannot be FrameSet!");
        return outFrame;
    }

    int64_t     seqID  = outFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX);
    OBFrameType type = outFrame->getType();
    auto ID = std::make_pair(seqID, type);

    std::lock_guard<std::recursive_mutex> lk(valueUpdateMutex_);

    // should filter but the current frame id does not matched
    if((selectedID_ != 0) && (seqID + 1) != selectedID_) {
        ID.first = (seqID == 0) ? 1 : 0;
        if(recentFrames_[ID])
            return recentFrames_[ID];
        return outFrame;
    }
    else {
        // should not filter, or the current frame id matches
        recentFrames_[ID] = outFrame;
        return outFrame;
    }
}

}  // namespace libobsensor
