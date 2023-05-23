#include "conn_posix.h"
#include "lwmavsdk.h"
#include "target.h"

#include <arpa/inet.h>
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

struct posix_udp_client_t
{
    int                fd;
    struct sockaddr_in addr;
};

static enum lwm_error_t
posix_udp_client_open(struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* params)
{
    ASSERT(ctx != NULL);
    ASSERT(params != NULL);
    ASSERT(params->type == LWM_CONN_TYPE_UDP_CLIENT);

    enum lwm_error_t    err = LWM_OK;
    struct posix_udp_client_t* udp
        = (struct posix_udp_client_t*)malloc(sizeof(struct posix_udp_client_t));
    if (udp == NULL)
    {
        WARN("posix_udp_client_open: unable to allocate udp context, err %s\n",
            strerror(errno));
        return LWM_ERR_NO_MEM;
    }

    udp->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp->fd < 0)
    {
        WARN("posix_udp_client_open: unable to open udp socket, err %s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    memset(&udp->addr, 0, sizeof(struct sockaddr_in));
    udp->addr.sin_family      = AF_INET;
    udp->addr.sin_port        = htons(params->params.udp.port);
    udp->addr.sin_addr.s_addr = inet_addr(params->params.udp.host);

    ctx->opaque = udp;
    return LWM_OK;

cleanup:
    if (udp->fd >= 0)
    {
        close(udp->fd);
    }
    free(udp);
    return err;
}

static void
posix_udp_client_close(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);

    struct posix_udp_client_t* udp = (struct posix_udp_client_t*)ctx->opaque;
    close(udp->fd);
    free(udp);
}

static enum lwm_error_t
posix_udp_client_send(struct lwm_conn_context_t* ctx, const uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct posix_udp_client_t* udp;
    udp = (struct posix_udp_client_t*)ctx->opaque;

    ssize_t rv = sendto(udp->fd, data, len, 0, (struct sockaddr*)&udp->addr,
        sizeof(struct sockaddr_in));
    if (rv < 0)
    {
        WARN("posix_udp_client_send: unable to send data, err %s\n", strerror(errno));
        return LWM_ERR_IO;
    }
    return LWM_OK;
}

static ssize_t
posix_udp_client_recv(struct lwm_conn_context_t* ctx, uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct posix_udp_client_t* udp;
    udp = (struct posix_udp_client_t*)ctx->opaque;

    ssize_t n = recvfrom(udp->fd, data, len, 0, NULL, NULL);
    if (n < 0)
    {
        WARN("posix_udp_client_recv: unable to recv data, err %s\n", strerror(errno));
        return -LWM_ERR_IO;
    }
    return n;
}

void
posix_udp_client_register(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);

    ctx->open  = posix_udp_client_open;
    ctx->close = posix_udp_client_close;
    ctx->send  = posix_udp_client_send;
    ctx->recv  = posix_udp_client_recv;
}
