#ifndef _LWMAVSDK_TARGET_H_
#define _LWMAVSDK_TARGET_H_

#if defined(POSIX_LIBC)

#if __cplusplus
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
using namespace std;

#else /* __cplusplus */
#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#endif /* __cplusplus */

#include <certikos/debug.h>
#include <time.h>
static inline uint64_t
time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}


#elif defined(CERTIKOS_USER)
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <certikos/debug.h>


#else
#error "Unsupported target"
#endif

static inline void lwm_puthex(const void * bytes, size_t len)
{
    uint8_t * p = (uint8_t *)bytes;
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02x", p[i]);
    }
#if defined(POSIX_LIBC)
    fflush(stdout);
#endif
}

static inline void lwm_putchars(const void * bytes, size_t len)
{
    uint8_t * p = (uint8_t *)bytes;
    for (size_t i = 0; i < len; ++i)
    {
        printf("%c", p[i]);
    }
#if defined(POSIX_LIBC)
    fflush(stdout);
#endif
}


#endif /* !_LWMAVSDK_TARGET_H_ */
