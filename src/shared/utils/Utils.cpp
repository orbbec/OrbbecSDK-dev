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

}  // namespace utils
}  // namespace libobsensor