#include "lwmavsdk.h"


enum lwm_error_t
lwm_conn_register(struct lwm_conn_context_t * ctx, enum lwm_conn_type_t type)
{
    switch (type)
    {
#if (POSIX_LIBC || defined(_MUSL_))
    case LWM_CONN_TYPE_SERIAL:
        posix_serial_register(ctx);
        return LWM_OK;
    case LWM_CONN_TYPE_UDP:
        posix_udp_register(ctx);
        return LWM_OK;
    case LWM_CONN_TYPE_UDP_CLIENT:
        posix_udp_client_register(ctx);
        return LWM_OK;
#endif
    case LWM_CONN_TYPE_PARTEE:
        certikos_user_partee_register(ctx);
        return LWM_OK;

#if (defined(_USER_) && defined(CERTIKOS_USER) && !defined(_MUSL_))
    case LWM_CONN_TYPE_CERTIKOS_SERIAL:
        certikos_user_serial_register(ctx);
        return LWM_OK;
    case LWM_CONN_TYPE_CERTIKOS_THINROS:
        certikos_user_thinros_register(ctx);
        return LWM_OK;
#endif
    default:
        PANIC("Unsupported connection type: %d\n", type);
        return LWM_ERR_NOT_SUPPORTED;
    }
    return LWM_ERR_NOT_SUPPORTED;
}
