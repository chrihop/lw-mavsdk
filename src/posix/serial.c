#include "lwmavsdk.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <termios.h>
#include <unistd.h>

struct posix_serial_t
{
    int                fd, epoll;
    struct epoll_event event[1];
};

static speed_t
posix_serial_baudrate(enum lwm_serial_baudrate_t baudrate)
{
    switch (baudrate)
    {
    case LWM_SERIAL_BAUDRATE_9600: return B9600;
    case LWM_SERIAL_BAUDRATE_19200: return B19200;
    case LWM_SERIAL_BAUDRATE_38400: return B38400;
    case LWM_SERIAL_BAUDRATE_57600: return B57600;
    case LWM_SERIAL_BAUDRATE_115200: return B115200;
    case LWM_SERIAL_BAUDRATE_230400: return B230400;
    case LWM_SERIAL_BAUDRATE_460800: return B460800;
    case LWM_SERIAL_BAUDRATE_921600: return B921600;
    default: return B115200;
    }
}

static const char *
posix_serial_baudrate_string(enum lwm_serial_baudrate_t baudrate)
{
    switch (baudrate)
    {
    case LWM_SERIAL_BAUDRATE_9600:   return "9600";
    case LWM_SERIAL_BAUDRATE_19200:  return "19200";
    case LWM_SERIAL_BAUDRATE_38400:  return "38400";
    case LWM_SERIAL_BAUDRATE_57600:  return "57600";
    case LWM_SERIAL_BAUDRATE_115200: return "115200";
    case LWM_SERIAL_BAUDRATE_230400: return "230400";
    case LWM_SERIAL_BAUDRATE_460800: return "460800";
    case LWM_SERIAL_BAUDRATE_921600: return "921600";
    default: return "unknown -> 115200";
    }
}


static enum lwm_error_t
posix_serial_open(
    struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* params)
{
    ASSERT(ctx != NULL);
    ASSERT(params != NULL);
    ASSERT(params->type == LWM_CONN_TYPE_SERIAL);
    ASSERT(params->params.serial.baudrate < MAX_LWM_SERIAL_BAUDRATE);

    enum lwm_error_t       err = LWM_OK;
    struct posix_serial_t* serial
        = (struct posix_serial_t*)malloc(sizeof(struct posix_serial_t));
    if (serial == NULL)
    {
        WARN("posix_serial_open: unable to allocate serial context, err %s\n",
            strerror(errno));
        return LWM_ERR_NO_MEM;
    }

    serial->fd
        = open(params->params.serial.device, O_RDWR | O_NOCTTY | O_NDELAY);
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
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    speed_t bd = posix_serial_baudrate(params->params.serial.baudrate);
    cfsetispeed(&tty, bd);
    cfsetospeed(&tty, bd);

    if (tcsetattr(serial->fd, TCSANOW, &tty) != 0)
    {
        WARN("posix_serial_open: unable to set serial device attributes, err "
             "%s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    serial->epoll = epoll_create1(0);
    if (serial->epoll < 0)
    {
        WARN("posix_serial_open: unable to create epoll, err %s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    struct epoll_event event;
    event.events  = EPOLLIN;
    event.data.fd = serial->fd;
    if (epoll_ctl(serial->epoll, EPOLL_CTL_ADD, serial->fd, &event) < 0)
    {
        WARN("posix_serial_open: unable to add serial fd to epoll, err %s\n",
            strerror(errno));
        err = LWM_ERR_IO;
        goto cleanup;
    }

    INFO("serial device %s:%s opened (fd = %d)\n", params->params.serial.device,
        posix_serial_baudrate_string(params->params.serial.baudrate),
        serial->fd);
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

    /* wait for event */
    int n_events = epoll_wait(serial->epoll, serial->event, 1, -1);
    if (n_events < 0)
    {
        WARN("posix_serial_recv: epoll_wait failed, err %s\n", strerror(errno));
        return -1;
    }

    if (n_events == 0 || serial->event[0].data.fd != serial->fd)
    {
        return 0;
    }

    ssize_t n = read(serial->fd, data, len);
    if (n < 0)
    {
        WARN("posix_serial_recv: unable to read from serial device, err %s\n",
            strerror(errno));
        return -1;
    }
    return n;
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
