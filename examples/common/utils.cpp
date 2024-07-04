#include "utils.hpp"
#include "utils_c.h"

namespace ob_sample_utils {
char waitForKeyPressed(uint32_t timeout_ms) {
    return ob_sample_utils_wait_for_key_press(timeout_ms);
}
}  // namespace ob_sample_utils