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

        int select_sequence_id = std::stoi(params[0]);
        if(select_sequence_id >= 0 && select_sequence_id <= 2) {
            if(static_cast<uint32_t>(select_sequence_id) != selectedID_) {
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
    static const std::string schema = "sequenceid, integer, 0, 2, 1, 0, frame data sequence id value";
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

    int64_t seq = selectedID_ == 0 ? 1 : 0;
    TRY_EXECUTE({ seq = outFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX); });
    OBFrameType type = outFrame->getType();
    auto        key  = std::make_pair(seq, type);

    std::lock_guard<std::recursive_mutex> lk(valueUpdateMutex_);
    // should filter but the current frame id does not matched
    if((selectedID_ != 0) && (seq + 1) != selectedID_) {
        key.first = (seq == 0) ? 1 : 0;
        if(recentFrames_[key]) {
            outFrame = recentFrames_[key];
        }
    }
    else {
        recentFrames_[key] = outFrame;
    }
    // should not filter, or the current frame id matches
    return outFrame;
}

}  // namespace libobsensor
