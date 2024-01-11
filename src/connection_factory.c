#include "lwmavsdk.h"

#if POSIX_LIBC
#include "conn_posix.h"
#elif CERTIKOS_USER
#include "conn_certikos_user.h"
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
    case LWM_CONN_TYPE_UDP_CLIENT:
        posix_udp_client_register(ctx);
        return LWM_OK;
    default:
        PANIC("Unsupported connection type: %d\n", type);
        return LWM_ERR_NOT_SUPPORTED;
    }
#elif CERTIKOS_USER
    switch (type)
    {
    case LWM_CONN_TYPE_CERTIKOS_SERIAL:
        certikos_user_serial_register(ctx);
        return LWM_OK;
    case LWM_CONN_TYPE_CERTIKOS_THINROS:
        certikos_user_thinros_register(ctx);
        return LWM_OK;
    default:
        WARN("Unsupported connection type: %d\n", type);
        return LWM_ERR_NOT_SUPPORTED;
    }
#endif
    return LWM_ERR_NOT_SUPPORTED;
}
