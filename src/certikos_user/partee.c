#include <certikos/partee.h>
#include <stdio.h>
#include <stdlib.h>
#include "lwmavsdk.h"

struct lwm_partee
{
    struct partee_publisher *pub;
    struct partee_subscriber *sub;
};

static ssize_t
certikos_user_partee_recv(
    struct lwm_conn_context_t* ctx, uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(len >= MAVLINK_MAX_PACKET_LEN);
    struct lwm_partee* lwm_partee = (struct lwm_partee*)ctx->opaque;
    while((len = partee_topic_read(lwm_partee->sub, data)) == 0)
    {
        // wait for data
    }

    return (ssize_t)len;

}

static enum lwm_error_t
certikos_user_partee_send(
    struct lwm_conn_context_t* ctx, const uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    struct lwm_partee* lwm_partee = (struct lwm_partee*)ctx->opaque;

    if(partee_topic_alloc_and_publish(lwm_partee->pub, data, len) != len)
    {
        return LWM_ERR_BAD_MESSAGE;
    }

    return LWM_OK;
}


static void
certikos_user_partee_close(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);
    struct lwm_partee* lwm_partee = (struct lwm_partee*)ctx->opaque;

    if(lwm_partee != NULL)
    {
        free(lwm_partee->pub);
        free(lwm_partee->sub);
        free(ctx->opaque);
    }
}



static enum lwm_error_t
certikos_user_partee_open(
    struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* conn)
{
    ASSERT(ctx != NULL);
    ASSERT(conn != NULL);
    ASSERT(conn->type == LWM_CONN_TYPE_PARTEE);


    ctx->opaque = calloc(1, sizeof(struct lwm_partee));
    if(ctx->opaque == NULL)
    {
        return LWM_ERR_NO_MEM;
    }

    struct lwm_partee* lwm_partee = (struct lwm_partee*)ctx->opaque;

    lwm_partee->pub = partee_create_publisher(
            conn->params.partee.publish_topic, 0, MAVLINK_MAX_PACKET_LEN);
    if(lwm_partee->pub == NULL)
    {
        free(ctx->opaque);
        ctx->opaque = NULL;
        return LWM_ERR_BAD_CONNECTION;
    }

    lwm_partee->sub = partee_create_subscription(
            conn->params.partee.subscribe_topic, NULL, 0, 0, MAVLINK_MAX_PACKET_LEN);
    if(lwm_partee->sub == NULL)
    {
        free(lwm_partee->pub);
        free(ctx->opaque);
        ctx->opaque = NULL;
        return LWM_ERR_BAD_CONNECTION;
    }


    return LWM_OK;
}



void
certikos_user_partee_register(struct lwm_conn_context_t *ctx)
{
    ASSERT(ctx != NULL);

    ctx->open  = certikos_user_partee_open;
    ctx->close = certikos_user_partee_close;
    ctx->send  = certikos_user_partee_send;
    ctx->recv  = certikos_user_partee_recv;
}
