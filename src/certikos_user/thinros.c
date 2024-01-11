#include <thinros.h>
#include <stdio.h>

#include "lwmavsdk.h"

struct lwm_thinros
{
    handler_t pub;

    uint8_t * recv_dst;
    size_t recv_len;
    ssize_t recv_res;
};
static struct lwm_thinros lwm_tros;

static void
receive_mavlink(void *msg)
{
    if(lwm_tros.recv_dst)
    {
        struct msg_mavlink_t * mav_msg = msg;

        if(mav_msg->len > lwm_tros.recv_len)
        {
            //throw
        }
        memcpy(lwm_tros.recv_dst, mav_msg->data, MIN(mav_msg->len, lwm_tros.recv_len));
        lwm_tros.recv_res = (ssize_t)mav_msg->len;
    }
}


static enum lwm_error_t
certikos_user_thinros_open(
    struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* params)
{
    ASSERT(ctx != NULL);
    ASSERT(params != NULL);
    ASSERT(params->type == LWM_CONN_TYPE_CERTIKOS_THINROS);

    lwm_tros.pub = ros_advertise(params->params.thinros.publish_topic);
    ros_subscribe(params->params.thinros.subscribe_topic, receive_mavlink);
    ctx->opaque = &lwm_tros;

    return LWM_OK;
}

static void
certikos_user_thinros_close(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
}

static enum lwm_error_t
certikos_user_thinros_send(
    struct lwm_conn_context_t* ctx, const uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct lwm_thinros * t = ctx->opaque;

    msg_mavlink_t mav_msg;
    mav_msg.len = len;
    memcpy(mav_msg.data, data, len);

    ros_publish(t->pub, &mav_msg, sizeof(mav_msg));

    return LWM_OK;
}

static ssize_t
certikos_user_thinros_recv(
    struct lwm_conn_context_t* ctx, uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct lwm_thinros * t = ctx->opaque;
    t->recv_dst = data;
    t->recv_len = len;
    t->recv_res = 0;
    ros_spin_one_message();
    return t->recv_res;
}



void
certikos_user_thinros_register(struct lwm_conn_context_t *ctx)
{
    ASSERT(ctx != NULL);

    ctx->open  = certikos_user_thinros_open;
    ctx->close = certikos_user_thinros_close;
    ctx->send  = certikos_user_thinros_send;
    ctx->recv  = certikos_user_thinros_recv;
}
