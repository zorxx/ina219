/*! \copyright 2025 Zorxx Software. All rights reserved.
 *  \brief portable helpers
 *  See LICENSE file in the root of this project source tree
 */
#ifndef _SYS_HELPERS_H
#define _SYS_HELPERS_H

/* Debug messaging */
#if defined(SYS_DEBUG_ENABLE) && defined(__linux__)
   #include <stdio.h>
   #define SERR(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
   #define SDBG(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")

#elif defined(SYS_DEBUG_ENABLE) && defined(ESP_PLATFORM)
   #include "esp_log.h"
   #define SERR(...) ESP_LOGE("SYS", __VA_ARGS__)
   #define SDBG(...) ESP_LOGI("SYS", __VA_ARGS__)

#else
   #define SERR(...)
   #define SDBG(...)

#endif

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#endif /* _SYS_HELPERS_H */
