#include "lwmavsdk.h"

static struct lwm_vehicle_t vehicle;

static enum lwm_action_continuation_t
callback_on_current_position(struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    ASSERT(action != NULL);
    ASSERT(param != NULL);

    mavlink_message_t* msg = param->detail.msg.msg;
    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(msg, &global_pos);
    INFO("Current Position: (%f, %f) at %f m\n",
        global_pos.lat / 10000000.0,
        global_pos.lon / 10000000.0,
        global_pos.alt / 1000.0);
    return LWM_ACTION_CONTINUE;
}

int
main(int argc, char** argv)
{
    lwm_vehicle_init(&vehicle);
    lwm_conn_open(&vehicle.conn, 14550);

    mavlink_message_t * msg;
    msg = lwm_command_request_message(&vehicle, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(msg, &global_pos);
    INFO("Home Position: (%f, %f) at %f m\n",
        global_pos.lat / 10000000.0,
        global_pos.lon / 10000000.0,
        global_pos.alt / 1000.0);

    struct lwm_command_t curr_pos;
    lwm_command_request_message_periodic(&vehicle, &curr_pos,
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000, callback_on_current_position);

    lwm_vehicle_spin(&vehicle);

    return 0;
}
