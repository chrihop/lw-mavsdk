#include "lwmavsdk.h"

struct lwm_vehicle_t vehicle;

uint8_t              rx_buf[1024];

int
main(int argc, char** argv)
{
    lwm_vehicle_init(&vehicle);
    lwm_conn_open(
        &vehicle.conn, LWM_CONN_TYPE_SERIAL, "/dev/ttyUART_IO2", 115200);

    ssize_t n;
    while (true)
    {
        n = vehicle.conn.recv(&vehicle.conn, rx_buf, sizeof(rx_buf));
        if (n < 0) {break;}
        lwm_puthex(rx_buf, n);
    }

    return 0;
}
