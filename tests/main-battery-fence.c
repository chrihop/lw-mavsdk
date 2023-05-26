#include "lwmavsdk.h"
#include "battery_fence.h"

static struct lwm_vehicle_t vehicle;

int
main(int argc, char** argv)
{
    enum lwm_error_t err;
    lwm_vehicle_init(&vehicle);
    const char * dev = "/dev/ttyAMA2";
    if (argc == 2)
    {
        dev = argv[1];
    }
    err = lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_SERIAL, dev, LWM_SERIAL_BAUDRATE_115200);

    if (err != LWM_OK)
    {
        PANIC("Failed to open connection\n");
    }

    battery_fence(&vehicle);

    return 0;
}
