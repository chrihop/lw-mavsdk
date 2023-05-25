#include "lwmavsdk.h"

struct lwm_vehicle_t vehicle;

static void ms_log(void * context, mavlink_message_t * msg)
{
    INFO("[%lu] Rx: msg id = %d len = %d seq = %d payload = ",
        time_us(), msg->msgid, msg->len, msg->seq);
    lwm_puthex(msg->payload64, msg->len);
    printf("\n");
}

int main(int argc, char ** argv)
{
    lwm_vehicle_init(&vehicle);
    lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_SERIAL, "/dev/ttyUART_IO2", 115200);

    lwm_microservice_t * log = lwm_microservice_create(&vehicle);
    log->handler = ms_log;
    lwm_microservice_add_to(&vehicle, MAVLINK_MSG_ID_HEARTBEAT, log);
    lwm_microservice_add_to(&vehicle, MAVLINK_MSG_ID_BATTERY_STATUS, log);
    lwm_vehicle_spin(&vehicle);

    return 0;
}
