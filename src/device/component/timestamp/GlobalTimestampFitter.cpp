#include "GlobalTimestampFitter.hpp"
#include "utils/Utils.hpp"
#include "logger/Logger.hpp"
#include "logger/LoggerInterval.hpp"
#include "InternalTypes.hpp"
#include "property/InternalProperty.hpp"

namespace libobsensor {
GlobalTimestampFitter::GlobalTimestampFitter(std::shared_ptr<IDevice> owner)
    : DeviceComponentBase(owner), sampleLoopExit_(false), linearFuncParam_({ 0, 0, 0, 0 }) {

    // todo: read config from xml

    // auto config = Context::getInstance()->getXmlConfig();

    // int value = 0;
    // if(config->getIntValue("Misc.GlobalTimestampFitterQueueSize", value) && value >= 4) {
    //     maxQueueSize_ = value;
    // }
    // value = 0;
    // if(config->getIntValue("Misc.GlobalTimestampFitterInterval", value) && value >= 100) {
    //     refreshIntervalMsec_ = value;
    // }

    sampleThread_ = std::thread(&GlobalTimestampFitter::fittingLoop, this);

    std::unique_lock<std::mutex> lock(linearFuncParamMutex_);
    linearFuncParamCondVar_.wait_for(lock, std::chrono::milliseconds(5000));

    LOG_DEBUG("GlobalTimestampFitter created: maxQueueSize_={}, refreshIntervalMsec_={}", maxQueueSize_, refreshIntervalMsec_);
}

GlobalTimestampFitter::~GlobalTimestampFitter() {
    sampleLoopExit_ = true;
    sampleCondVar_.notify_one();
    if(sampleThread_.joinable()) {
        sampleThread_.join();
    }
}

LinearFuncParam GlobalTimestampFitter::getLinearFuncParam() {
    std::unique_lock<std::mutex> lock(linearFuncParamMutex_);
    return linearFuncParam_;
}

void GlobalTimestampFitter::reFitting() {
    std::unique_lock<std::mutex> lock(sampleMutex_);
    samplingQueue_.clear();
    sampleCondVar_.notify_one();
}

void GlobalTimestampFitter::pause() {
    sampleLoopExit_ = true;
    sampleCondVar_.notify_one();
    if(sampleThread_.joinable()) {
        sampleThread_.join();
    }
}

void GlobalTimestampFitter::resume() {
    sampleLoopExit_ = false;
    sampleThread_   = std::thread(&GlobalTimestampFitter::fittingLoop, this);
}

void GlobalTimestampFitter::fittingLoop() {
    std::unique_lock<std::mutex> lock(sampleMutex_);
    const int                    MAX_RETRY_COUNT = 5;

    int retryCount = 0;
    do {
        if(samplingQueue_.size() > maxQueueSize_) {
            samplingQueue_.pop_front();
        }

        uint64_t     sysTspUsec = 0;
        OBDeviceTime devTime;

        BEGIN_TRY_EXECUTE({
            auto owner            = getOwner();
            auto propertyAccessor = owner->getPropertyAccessor();

            auto sysTsp1Usec = utils::getNowTimesUs();
            devTime          = propertyAccessor->getStructureDataT<OBDeviceTime>(OB_STRUCT_DEVICE_TIME);
            auto sysTsp2Usec = utils::getNowTimesUs();
            sysTspUsec       = (sysTsp2Usec + sysTsp1Usec) / 2;
            devTime.rtt      = sysTsp2Usec - sysTsp1Usec;
            if(devTime.rtt > 10000) {
                throw io_exception(utils::string::to_string() << "Get device time rtt is too large! rtt=" << devTime.rtt);
            }

            LOG_TRACE("sys={}, dev={}, rtt={}", sysTspUsec, devTime.time, devTime.rtt);
        })
        CATCH_EXCEPTION_AND_EXECUTE({
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            retryCount++;
            continue;
        })

        // Successfully obtain timestamp, the number of retries is reset to zero
        retryCount = 0;

        // Clearing and refitting when the timestamp is out of order
        if(!samplingQueue_.empty() && (devTime.time < samplingQueue_.back().deviceTimestamp)) {
            samplingQueue_.clear();
        }

        samplingQueue_.push_back({ sysTspUsec, devTime.time });

        if(samplingQueue_.size() < 10) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // Use the first set of data as offset to prevent overflow during calculation
        uint64_t offset_x = samplingQueue_.front().deviceTimestamp;
        uint64_t offset_y = samplingQueue_.front().systemTimestamp;
        double   Ex       = 0;
        double   Exx      = 0;
        double   Ey       = 0;
        double   Exy      = 0;
        auto     it       = samplingQueue_.begin();
        while(it != samplingQueue_.end()) {
            auto systemTimestamp = it->systemTimestamp - offset_y;
            auto deviceTimestamp = it->deviceTimestamp - offset_x;
            Ex += deviceTimestamp;
            Exx += deviceTimestamp * deviceTimestamp;
            Ey += systemTimestamp;
            Exy += deviceTimestamp * systemTimestamp;
            it++;
        }

        {
            std::unique_lock<std::mutex> linearFuncParamLock(linearFuncParamMutex_);
            // Linear regression to find a and b: y=ax+b
            linearFuncParam_.coefficientA = (Exy * samplingQueue_.size() - Ex * Ey) / (samplingQueue_.size() * Exx - Ex * Ex);
            linearFuncParam_.constantB  = (Exx * Ey - Exy * Ex) / (samplingQueue_.size() * Exx - Ex * Ex) + offset_y - linearFuncParam_.coefficientA * offset_x;
            linearFuncParam_.checkDataX = devTime.time;
            linearFuncParam_.checkDataY = sysTspUsec;

            // auto fixDevTsp = (double)devTime *linearFuncParam_.coefficientA + linearFuncParam_.constantB;
            // auto fixDiff   = fixDevTsp -sysTspUsec;
            // LOG_TRACE("a = {}, b = {}, fix={}, diff={}", linearFuncParam_.coefficientA, linearFuncParam_.constantB, fixDevTsp, fixDiff);

            LOG_DEBUG_INTVL("GlobalTimestampFitter update: coefficientA = {}, constantB = {}", linearFuncParam_.coefficientA, linearFuncParam_.constantB);
            linearFuncParamCondVar_.notify_all();
        }

        sampleCondVar_.wait_for(lock, std::chrono::milliseconds(refreshIntervalMsec_));
    } while(!sampleLoopExit_ && retryCount <= MAX_RETRY_COUNT);

    if(retryCount > MAX_RETRY_COUNT) {
        LOG_ERROR("GlobalTimestampFitter fittingLoop retry count exceed max retry count!");
    }

    LOG_DEBUG("GlobalTimestampFitter fittingLoop exit");
}

}  // namespace libobsensor