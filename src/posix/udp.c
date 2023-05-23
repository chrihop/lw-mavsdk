#include "conn_posix.h"
#include "lwmavsdk.h"
#include "target.h"

#include <arpa/inet.h>
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

struct posix_udp_t
{
    int                fd;
    struct sockaddr_in client;
    bool               has_client;
};

static enum lwm_error_t
posix_udp_open(
    struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* params)
{
    ASSERT(ctx != NULL);
    ASSERT(params != NULL);
    ASSERT(params->type == LWM_CONN_TYPE_UDP);

    enum lwm_error_t           err = LWM_OK;
    struct posix_udp_t* udp
        = (struct posix_udp_t*)malloc(sizeof(struct posix_udp_t));
    if (udp == NULL)
    {
        WARN("posix_udp_open: unable to allocate udp context, err %s\n",
            strerror(errno));
        return LWM_ERR_NO_MEM;
    }

    udp->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp->fd < 0)
    {
        WARN("posix_udp_open: unable to open udp socket, err %s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(params->params.udp.port);
    addr.sin_addr.s_addr = INADDR_ANY;

    int rv = bind(udp->fd, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
    if (rv < 0)
    {
        WARN("posix_udp_open: unable to bind udp socket, err %s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    uint8_t   buf[1];
    socklen_t caddr_len = sizeof(udp->client);
    ssize_t   n = recvfrom(udp->fd, buf, sizeof(buf), 0,
                  (struct sockaddr*)&udp->client, &caddr_len);
    INFO("Wait for UDP client ...\n");
    if (n <= 0)
    {
        WARN("posix_udp_open: unable to receive udp packet, err %s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    udp->has_client = true;
    INFO("UDP connection: %d <--> %s:%d\n", params->params.udp.port,
        inet_ntoa(udp->client.sin_addr), ntohs(udp->client.sin_port));

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
posix_udp_close(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);

    struct posix_udp_t* udp = (struct posix_udp_t*)ctx->opaque;
    close(udp->fd);
    free(udp);
}

static enum lwm_error_t
posix_udp_send(
    struct lwm_conn_context_t* ctx, const uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct posix_udp_t* udp;
    udp = (struct posix_udp_t*)ctx->opaque;

    ssize_t rv = sendto(udp->fd, data, len, 0, (struct sockaddr *) &udp->client,
        sizeof(struct sockaddr_in));
    if (rv < 0)
    {
        WARN("posix_udp_send: unable to send data, err %s\n",
            strerror(errno));
        return LWM_ERR_IO;
    }
    return LWM_OK;
}

static ssize_t
posix_udp_recv(struct lwm_conn_context_t* ctx, uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct posix_udp_t* udp;
    udp = (struct posix_udp_t*)ctx->opaque;

    socklen_t caddr_len = sizeof(udp->client);
    ssize_t n = recvfrom(udp->fd, data, len, 0, (struct sockaddr *) &udp->client, &caddr_len);
    if (n < 0)
    {
        WARN("posix_udp_recv: unable to recv data, err %s\n",
            strerror(errno));
        return -LWM_ERR_IO;
    }
    return n;
}

void
posix_udp_register(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);

    ctx->open  = posix_udp_open;
    ctx->close = posix_udp_close;
    ctx->send  = posix_udp_send;
    ctx->recv  = posix_udp_recv;
}
