#include <syscall.h>

#include "lwmavsdk.h"

struct lwm_certikos_user_serial_t
{
    size_t fd;
};

static struct lwm_certikos_user_serial_t serial;

static enum lwm_error_t
certikos_user_serial_open(
    struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* params)
{
    ASSERT(ctx != NULL);
    ASSERT(params != NULL);
    ASSERT(params->type == LWM_CONN_TYPE_CERTIKOS_SERIAL);

    size_t               dev = params->params.certikos_serial.device;

    enum syscall_error_t err
        = sys_device_control(dev, DEV_CONSOLE_NON_BLOCKING, 0, &serial.fd);
    if (err != 0)
    {
        WARN("Failed to open serial device %d: %s(%d)", dev, syscall_err(err),
            err);
        return LWM_ERR_IO;
    }
    ctx->opaque = &serial;

    return LWM_OK;
}

static void
certikos_user_serial_close(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
}

static enum lwm_error_t
certikos_user_serial_send(
    struct lwm_conn_context_t* ctx, const uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct lwm_certikos_user_serial_t* s
        = (struct lwm_certikos_user_serial_t*)ctx->opaque;
    writes(s->fd, data, len);
    return LWM_OK;
}

static ssize_t
certikos_user_serial_recv(
    struct lwm_conn_context_t* ctx, uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct lwm_certikos_user_serial_t* s
        = (struct lwm_certikos_user_serial_t*)ctx->opaque;
    size_t n = reads(s->fd, data, len);
    return (ssize_t)n;
}

void
certikos_user_serial_register(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);

    ctx->open  = certikos_user_serial_open;
    ctx->close = certikos_user_serial_close;
    ctx->send  = certikos_user_serial_send;
    ctx->recv  = certikos_user_serial_recv;
}
