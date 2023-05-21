#include "lwmavsdk.h"

#if POSIX_LIBC
#include "conn_posix.h"
#elif CERTIKOS_USER
#endif

enum lwm_error_t
lwm_conn_register(struct lwm_conn_context_t * ctx, enum lwm_conn_type_t type)
{
#if POSIX_LIBC
    switch (type)
    {
    case LWM_CONN_TYPE_SERIAL:
        posix_serial_register(ctx);
        return LWM_OK;
    case LWM_CONN_TYPE_UDP:
        posix_udp_register(ctx);
        return LWM_OK;
    default:
        PANIC("Unsupported connection type: %d\n", type);
        return LWM_ERR_NOT_SUPPORTED;
    }
#elif CERTIKOS_USER
    switch (type)
    {}
#endif
    return LWM_ERR_NOT_SUPPORTED;
}
