#include "conn_posix.h"
#include "lwmavsdk.h"
#include "target.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

struct posix_serial_t
{
    int                fd;
};

static enum lwm_error_t
posix_serial_open(
    struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* params)
{
    ASSERT(ctx != NULL);
    ASSERT(params != NULL);
    ASSERT(params->type == LWM_CONN_TYPE_SERIAL);

    enum lwm_error_t       err = LWM_OK;
    struct posix_serial_t* serial
        = (struct posix_serial_t*)malloc(sizeof(struct posix_serial_t));
    if (serial == NULL)
    {
        WARN("posix_serial_open: unable to allocate serial context, err %s\n",
            strerror(errno));
        return LWM_ERR_NO_MEM;
    }

    serial->fd = open(params->params.serial.device, O_RDWR | O_NOCTTY);
    if (serial->fd < 0)
    {
        WARN("posix_serial_open: unable to open serial device %s, err %s\n",
            params->params.serial.device, strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial->fd, &tty) != 0)
    {
        WARN("posix_serial_open: unable to get serial device attributes, err "
             "%s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    cfmakeraw(&tty);
    tty.c_cflag += CS8 | CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    cfsetispeed(&tty, params->params.serial.baudrate);
    cfsetospeed(&tty, params->params.serial.baudrate);

    if (tcsetattr(serial->fd, TCSANOW, &tty) != 0)
    {
        WARN("posix_serial_open: unable to set serial device attributes, err "
             "%s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    ctx->opaque = serial;
    return LWM_OK;

cleanup:
    if (serial->fd > 0)
    {
        close(serial->fd);
    }
    free(serial);
    return err;
}

static void
posix_serial_close(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);

    struct posix_serial_t* serial = (struct posix_serial_t*)ctx->opaque;
    close(serial->fd);
    free(serial);
}

static enum lwm_error_t
posix_serial_send(
    struct lwm_conn_context_t* ctx, const uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct posix_serial_t* serial = (struct posix_serial_t*)ctx->opaque;
    ssize_t                ret    = write(serial->fd, data, len);
    if (ret < 0)
    {
        WARN("posix_serial_send: unable to write to serial device, err %s\n",
            strerror(errno));
        return LWM_ERR_IO;
    }
    return LWM_OK;
}

static ssize_t
posix_serial_recv(struct lwm_conn_context_t* ctx, uint8_t* data, size_t len)
{
    ASSERT(ctx != NULL);
    ASSERT(ctx->opaque != NULL);
    ASSERT(data != NULL);
    ASSERT(len > 0);

    struct posix_serial_t* serial = (struct posix_serial_t*)ctx->opaque;
    ssize_t                ret    = read(serial->fd, data, len);
    if (ret < 0)
    {
        WARN("posix_serial_recv: unable to read from serial device, err %s\n",
            strerror(errno));
        return -1;
    }
    return ret;
}

void
posix_serial_register(struct lwm_conn_context_t* ctx)
{
    ASSERT(ctx != NULL);

    ctx->open  = posix_serial_open;
    ctx->close = posix_serial_close;
    ctx->send  = posix_serial_send;
    ctx->recv  = posix_serial_recv;
}
