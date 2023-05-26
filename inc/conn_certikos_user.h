#ifndef _LWMAVSDK_CONN_POSIX_H_
#define _LWMAVSDK_CONN_POSIX_H_

#include "lwmavsdk.h"

#if __cplusplus
extern "C" {
#endif

void certikos_user_serial_register(struct lwm_conn_context_t* ctx);

#if __cplusplus
};
#endif

#endif /* !_LWMAVSDK_CONN_POSIX_H_ */
