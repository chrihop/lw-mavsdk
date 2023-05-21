#ifndef _LWMAVSDK_TARGET_H_
#define _LWMAVSDK_TARGET_H_

#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

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

#include <execinfo.h>

static inline void
print_backtrace(void)
{
    void*  callstack[32];
    int    i, frames = backtrace(callstack, 32);
    char** strs = backtrace_symbols(callstack, frames);
    for (i = 0; i < frames; ++i)
    {
        if (i == 2)
        {
            printf(" ===> ");
        }
        else
        {
            printf("      ");
        }
        printf("%s\n", strs[i]);
    }
    free(strs);
}

#include <time.h>
static inline uint64_t
time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

#ifndef INFO
#define INFO(fmt, ...) fprintf(stderr, "[I] " fmt, ##__VA_ARGS__)
#endif

#ifndef WARN
#define WARN(fmt, ...)                                                         \
    fprintf(stderr, "[W] %s:%u (in %s()) " fmt, __FILE__, __LINE__,            \
        __FUNCTION__, ##__VA_ARGS__)
#endif

#ifndef PANIC
#define PANIC(fmt, ...)                                                        \
    do                                                                         \
    {                                                                          \
        fprintf(stderr, "[P] %s:%u (in %s()) " fmt, __FILE__, __LINE__,        \
            __FUNCTION__, ##__VA_ARGS__);                                      \
        print_backtrace();                                                     \
        abort();                                                               \
    } while (0)
#endif

#ifndef ASSERT
#define ASSERT(x)                                                              \
    do                                                                         \
    {                                                                          \
        if (unlikely(!(x)))                                                    \
        {                                                                      \
            fprintf(stderr, "[A] %s:%u (in %s()) " #x, __FILE__, __LINE__,     \
                __FUNCTION__);                                                 \
            print_backtrace();                                                 \
            abort();                                                           \
        }                                                                      \
    } while (0)
#endif

#elif defined(CERTIKOS_USER)

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
}

static inline void lwm_putchars(const void * bytes, size_t len)
{
    uint8_t * p = (uint8_t *)bytes;
    for (size_t i = 0; i < len; ++i)
    {
        printf("%c", p[i]);
    }
}


#endif /* !_LWMAVSDK_TARGET_H_ */
