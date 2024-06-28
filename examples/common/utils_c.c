#include "utils_c.h"
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
void print_error(const char *message) {
    printf("%s\n", message);
}
#ifdef __cplusplus
}
#endif
