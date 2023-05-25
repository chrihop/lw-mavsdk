#include "lwmavsdk.h"

struct lwm_vehicle_t vehicle;

uint8_t              rx_buf[1024];

int
main(int argc, char** argv)
{
    lwm_vehicle_init(&vehicle);

    const char * dev = "/dev/ttyAMA2";
    if (argc == 2)
    {
        dev = argv[1];
    }
    lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_SERIAL, dev, LWM_SERIAL_BAUDRATE_115200);

    ssize_t n;
    while (true)
    {
        n = vehicle.conn.recv(&vehicle.conn, rx_buf, 1024);
        if (n < 0) {break;}
        lwm_puthex(rx_buf, n);
        fflush(stdout);
    }

    return 0;
}
