#include "Utils.hpp"

#ifdef _WINDOWS
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <chrono>
#include <logger/Logger.hpp>

namespace libobsensor {
namespace utils {

uint64_t getNowTimesMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
uint64_t getNowTimesUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void sleepMs(uint64_t msec) {
#ifdef WIN32
    Sleep((DWORD)msec);
#else
    sleep((double)msec / 1000);
#endif
}

bool checkJpgImageData(const uint8_t *data, size_t dataLen) {
    bool validImage = dataLen >= 2 && data[0] == 0xFF && data[1] == 0xD8;
    if(validImage) {
        // 为了解决MJPG数据尾部（0xFF 0xD9）由于对齐等情况未按照标准MJPG排列导致的误判情况
        size_t startIndex = dataLen - 1;
        size_t endIndex   = startIndex - 10;
        for(size_t index = startIndex; index > endIndex && index > 0; index--) {
            if(data[index] != 0x00) {
                if(data[index] == 0xD9 && data[index - 1] == 0xFF) {
                    validImage = true;
                }
                else {
                    LOG_DEBUG("check mjpg end flag failed:data[index]:{0},data[index - 1]:{1}", data[index], data[index - 1]);
                    validImage = false;
                }
                break;
            }
        }
    }
    return validImage;
}

bool getFirmwareVersionInt(std::string fwVersionStr, int &fwVersion) {
    int  dotCount     = 0;
    char buf[16]      = { 0 };
    int  bufIndex     = 0;
    int  calFwVersion = 0;
    for(int i = 0; i < fwVersionStr.size(); i++) {
        const char c = fwVersionStr[i];
        if(isdigit(c) && bufIndex < sizeof(buf)) {
            buf[bufIndex] = c;
            bufIndex++;
        }
        if('.' == c) {
            buf[sizeof(buf) - 1] = '\0';
            if(strlen(buf) > 0) {
                int value = atoi(buf);
                // 版本号只有两位
                if(value >= 100) {
                    LOG_ERROR("bad fwVersion: {}", fwVersionStr);
                    return false;
                }

                if(dotCount == 0) {  // 主版本号
                    calFwVersion += 10000 * value;
                }
                else if(dotCount == 1) {  // 次版本号
                    calFwVersion += 100 * value;
                }
                else {
                    LOG_ERROR("bad fwVersion: {}", fwVersionStr);
                    return false;
                }

                dotCount++;
                bufIndex = 0;
                memset(buf, 0, sizeof(buf));
            }
        }
    }

    buf[sizeof(buf) - 1] = '\0';
    if(strlen(buf) > 0 && strlen(buf) <= 2 && dotCount == 2) {
        int value = atoi(buf);
        calFwVersion += value;
    }

    if(calFwVersion == 0 || dotCount < 2) {
        LOG_ERROR("bad fwVersion: {}, parse digital version failed", fwVersionStr);
        return false;
    }

    fwVersion = calFwVersion;
    return true;
}

}  // namespace utils
}  // namespace libobsensor