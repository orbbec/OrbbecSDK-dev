#include "utils.hpp"
#include "utils_c.h"
#include "utils_key.h"

#include <chrono>

namespace ob_sample_utils {
char waitForKeyPressed(uint32_t timeout_ms) {
    return ob_sample_utils_wait_for_key_press(timeout_ms);
}

uint64_t getNowTimesMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int getInputOption() {
    char inputOption = ob_sample_utils::waitForKeyPressed();
    if(inputOption == ESC_KEY) {
        return -1;
    }
    return inputOption - '0';
}

}  // namespace ob_sample_utils

uint64_t ob_sample_utils_get_current_timestamp_ms(){
    return ob_sample_utils::getNowTimesMs();
}