#include "lwmavsdk.h"

struct lwm_vehicle_t vehicle;

static void
ms_log(void* context, mavlink_message_t* msg)
{
    INFO("[%lu] Rx: msg id = %d len = %d seq = %d ", time_us(),
        msg->msgid, msg->len, msg->seq);

    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_BATTERY_STATUS:
        mavlink_battery_status_t battery_status;
        mavlink_msg_battery_status_decode(msg, &battery_status);
        INFO("Battery status: {current consumed %d mAh, remaining %d %%, energy consumed %d hJ}\n",
            battery_status.current_consumed, battery_status.battery_remaining,
            battery_status.energy_consumed);
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        INFO("Heartbeat: {type %d, autopilot %d, base_mode %d, custom_mode %d, system_status %d, mavlink_version %d}\n",
            heartbeat.type, heartbeat.autopilot, heartbeat.base_mode,
            heartbeat.custom_mode, heartbeat.system_status,
            heartbeat.mavlink_version);
        break;
    default:
        printf("payload = ");
        lwm_puthex(msg->payload64, msg->len);
        printf("\n");
    }
}

int
main(int argc, char** argv)
{
    lwm_vehicle_init(&vehicle);
    const char * ip = "192.168.10.193";
    if (argc == 2)
    {
        ip = argv[1];
    }

    lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_UDP_CLIENT, ip, 14550);

    lwm_microservice_t* log = lwm_microservice_create(&vehicle);
    log->handler            = ms_log;
    lwm_microservice_add_to(&vehicle, MAVLINK_MSG_ID_HEARTBEAT, log);
    lwm_microservice_add_to(&vehicle, MAVLINK_MSG_ID_BATTERY_STATUS, log);
    lwm_vehicle_spin(&vehicle);

    return 0;
}
