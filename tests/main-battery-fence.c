#include "lwmavsdk.h"
#include <math.h>

static struct lwm_vehicle_t vehicle;

double get_distance_meters(mavlink_global_position_int_t* pos1, mavlink_global_position_int_t* pos2) {

    double dlat = pos1->lat - pos2->lat;
    double dlong = pos1->lon - pos2->lon;
    
    double dist = sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5;
    return dist;
}

static void
callback_on_current_position(void * context, mavlink_message_t * msg)
{
    ASSERT(msg != NULL);
    if (msg->msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        return ;
    }

    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(msg, &global_pos);
    INFO("Current Position: (%f, %f) at %f m\n",
        global_pos.lat / 10000000.0,
        global_pos.lon / 10000000.0,
        global_pos.alt / 1000.0);
}

static void
    callback_on_battery_status(void * context, mavlink_message_t * msg)
{
    ASSERT(msg != NULL);

    if (msg->msgid != MAVLINK_MSG_ID_BATTERY_STATUS) {
        return ;
    }

    mavlink_battery_status_t battery_status;
    mavlink_msg_battery_status_decode(msg, &battery_status);
    INFO("Battery Status: %d%%\n", battery_status.battery_remaining);
}

int
main(int argc, char** argv)
{
    lwm_vehicle_init(&vehicle);
    lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_UDP, "127.0.0.1", 14550);

    mavlink_message_t * msg;
    msg = lwm_command_request_message(&vehicle, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(msg, &global_pos);
    INFO("Home Position: (%f, %f) at %f m\n",
        global_pos.lat / 10000000.0,
        global_pos.lon / 10000000.0,
        global_pos.alt / 1000.0);

    struct lwm_microservice_t * curr_pos = lwm_microservice_create(&vehicle);
    curr_pos->handler = callback_on_current_position;
    curr_pos->context = NULL;
    lwm_microservice_add_to(&vehicle, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, curr_pos);

    struct lwm_microservice_t * battery = lwm_microservice_create(&vehicle);
    battery->handler = callback_on_battery_status;
    battery->context = NULL;
    lwm_microservice_add_to(&vehicle, MAVLINK_MSG_ID_BATTERY_STATUS, battery);

    lwm_vehicle_spin(&vehicle);

    return 0;
}
