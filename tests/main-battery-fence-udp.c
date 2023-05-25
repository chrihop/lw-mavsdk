#include "lwmavsdk.h"
#include "battery_fence.h"

static struct lwm_vehicle_t vehicle;

int
main(int argc, char** argv)
{
    enum lwm_error_t err;
    lwm_vehicle_init(&vehicle);

    err = lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_UDP, 14550);

    if (err != LWM_OK)
    {
        PANIC("Failed to open connection\n");
    }

    battery_fence(&vehicle);

    return 0;
}
