#include "lwmavsdk.h"

static struct lwm_vehicle_t vehicle;

/**
 * @brief Battery fence
 * @details see battery-fence-common.c
 * @param vehicle
 */
extern void battery_fence(struct lwm_vehicle_t* vehicle);

int
main(int argc, char** argv)
{
    enum lwm_error_t err;
    lwm_vehicle_init(&vehicle);
    err = lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_SERIAL, "/dev/ttyAMA2", 115200);
    if (err != LWM_OK)
    {
        PANIC("Failed to open connection\n");
    }

    battery_fence(&vehicle);

    return 0;
}
