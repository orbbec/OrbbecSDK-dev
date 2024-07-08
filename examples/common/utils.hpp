#pragma once
#include <stdint.h>
#include "utils_types.h"

namespace ob_smpl {
char waitForKeyPressed(uint32_t timeout_ms = 0);

uint64_t getNowTimesMs();

int getInputOption();

}