#include "lwmavsdk.h"
#include "target.h"
#include <v2.0/ardupilotmega/mavlink.h>

static void
lwm_read_buffer_init(struct lwm_read_buffer_t* buf)
{
    buf->len = 0;
    buf->pos = 0;
}

static uint8_t*
lwm_read_buffer_data(struct lwm_read_buffer_t* buf)
{
    return buf->buffer;
}

static void
lwm_read_buffer_set(struct lwm_read_buffer_t* buf, size_t len)
{
    buf->len = len;
    buf->pos = 0;
}

static bool
lwm_read_buffer_empty(struct lwm_read_buffer_t* buf)
{
    return buf->pos >= buf->len;
}

static void
lwm_conn_init(struct lwm_conn_context_t* ctx)
{
    ctx->status = LWM_CONN_STATUS_CLOSED;
    ctx->opaque = NULL;
    ctx->open   = NULL;
    ctx->close  = NULL;
    ctx->send   = NULL;
    ctx->recv   = NULL;
    memset(&ctx->rx_status, 0, sizeof(ctx->rx_status));
}

enum lwm_error_t
lwm_conn_open(struct lwm_conn_context_t* ctx, enum lwm_conn_type_t type, ...)
{
    struct lwm_conn_params_t params;
    enum lwm_error_t         err;

    lwm_conn_init(ctx);
    if (lwm_conn_register(ctx, type) != LWM_OK)
    {
        return LWM_ERR_NOT_SUPPORTED;
    }

    params.type = type;
    va_list args;
    va_start(args, type);

    switch (type)
    {
    case LWM_CONN_TYPE_UDP:
    {
        params.params.udp.host = va_arg(args, const char*);
        params.params.udp.port = va_arg(args, int);
        break;
    }
    case LWM_CONN_TYPE_TCP:
    {
        params.params.tcp.host = va_arg(args, const char*);
        params.params.tcp.port = va_arg(args, int);
        break;
    }
    case LWM_CONN_TYPE_SERIAL:
    {
        params.params.serial.device   = va_arg(args, const char*);
        params.params.serial.baudrate = va_arg(args, uint32_t);
        break;
    }
    case LWM_CONN_TYPE_CERTIKOS_SERIAL:
    {
        params.params.certikos_serial.device   = va_arg(args, uint32_t);
        params.params.certikos_serial.baudrate = va_arg(args, uint32_t);
        break;
    }
    default:
    {
        va_end(args);
        return LWM_ERR_BAD_PARAM;
    }
    }
    va_end(args);

    ctx->type = type;
    err       = ctx->open(ctx, &params);
    if (err != LWM_OK)
    {
        return err;
    }
    ctx->status = LWM_CONN_STATUS_OPEN;
    return LWM_OK;
}

enum lwm_error_t
lwm_conn_send(struct lwm_conn_context_t* ctx, mavlink_message_t* msg)
{
    ASSERT(ctx != NULL && ctx->send != NULL
        && ctx->status == LWM_CONN_STATUS_OPEN);

    size_t  min_len   = mavlink_min_message_length(msg);
    uint8_t crc_extra = mavlink_get_crc_extra(msg);
    mavlink_finalize_message(
        msg, msg->sysid, msg->compid, min_len, msg->len, crc_extra);
    size_t len = mavlink_msg_to_send_buffer(ctx->output, msg);
    return ctx->send(ctx, ctx->output, len);
}

static void
lwm_conn_packet_drop_analyzer(
    struct lwm_conn_context_t* ctx, size_t pos, size_t len)
{
    struct lwm_read_buffer_t* input = &ctx->input;
    ASSERT(pos + len <= input->len);

    WARN("packet raw format (%lu bytes): ", len);
    lwm_puthex(&input[pos], len);
    printf("\n");

    WARN("msg {id = %d, len = %d, crc = %d, sysid = %d, compid = %d}\n",
        ctx->rx_message.msgid, ctx->rx_message.len, ctx->rx_message.checksum,
        ctx->rx_message.sysid, ctx->rx_message.compid);
}

enum lwm_error_t
lwm_conn_recv(struct lwm_conn_context_t* ctx, mavlink_message_t* msg)
{
    ASSERT(ctx != NULL && ctx->send != NULL
        && ctx->status == LWM_CONN_STATUS_OPEN);

    struct lwm_read_buffer_t* input = &ctx->input;
    if (!lwm_read_buffer_empty(input))
    {
        size_t drop      = ctx->rx_status.packet_rx_drop_count;
        size_t start_pos = input->pos;
        for (; input->pos < input->len; input->pos++)
        {
            uint8_t c = input->buffer[input->pos];
            if (mavlink_parse_char(
                    MAVLINK_COMM_0, c, &ctx->rx_message, &ctx->rx_status))
            {
                memcpy(msg, &ctx->rx_message, sizeof(mavlink_message_t));
                input->pos++;
                return LWM_OK;
            }
            if (ctx->rx_status.packet_rx_drop_count > drop)
            {
                WARN("%lu packet (%lu bytes) dropped\n",
                    ctx->rx_status.packet_rx_drop_count - drop,
                    input->pos - start_pos);
                drop      = ctx->rx_status.packet_rx_drop_count;
                start_pos = input->pos;
                continue;
            }
        }
    }
    ASSERT(lwm_read_buffer_empty(input));
    ssize_t len = ctx->recv(ctx, input->buffer, LWM_READ_BUFFER_SIZE - 1);
    if (len <= 0)
    {
        return LWM_ERR_IO;
    }
    lwm_read_buffer_set(input, len);
    //    lwm_puthex(input->buffer, len);
    return LWM_ERR_NO_DATA;
}

void
lwm_conn_close(struct lwm_conn_context_t* ctx)
{
    ctx->close(ctx);
}