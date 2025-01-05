#include "lwmavsdk.h"

static struct lwm_vehicle_t vehicle;

static enum lwm_error_t
get_home_position_exec(struct lwm_action_t* action, void* data)
{
    ASSERT(action != NULL);
    ASSERT(action->data == NULL);
    ASSERT(action->vehicle != NULL);

    mavlink_command_long_t cmd;
    cmd.target_system    = 1;
    cmd.target_component = 1;
    cmd.command          = MAV_CMD_GET_HOME_POSITION;
    cmd.confirmation     = 0;
    cmd.param1           = 0;
    cmd.param2           = 0;
    cmd.param3           = 0;
    cmd.param4           = 0;
    cmd.param5           = 0;
    cmd.param6           = 0;
    cmd.param7           = 0;

    mavlink_message_t msg;
    mavlink_msg_command_long_encode(255, 1, &msg, &cmd);

    return lwm_conn_send(&action->vehicle->conn, &msg);
}

struct pos_t
{
    int32_t lat;
    int32_t lon;
    int32_t alt;
};

static struct pos_t home_pos;

static enum lwm_action_continuation_t
get_home_position_then(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    ASSERT(action != NULL);
    ASSERT(action->data == NULL);
    ASSERT(action->vehicle != NULL);
    ASSERT(param != NULL);

    mavlink_message_t* msg = param->detail.msg.msg;
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        if (ack.command == MAV_CMD_GET_HOME_POSITION
            && ack.result != MAV_RESULT_ACCEPTED)
        {
            WARN("MAV_CMD_GET_HOME_POSITION failed: %d\n", ack.result);
            return LWM_ACTION_STOP;
        }
        return LWM_ACTION_CONTINUE;
    }
    break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(msg, &pos);

        if (pos.time_boot_ms > 0 && pos.alt > 0)
        {
            home_pos.lat = pos.lat;
            home_pos.lon = pos.lon;
            home_pos.alt = pos.alt;
            action->result = &home_pos;
            return LWM_ACTION_STOP;
        }
        return LWM_ACTION_CONTINUE;
    }
    break;
    default: return LWM_ACTION_CONTINUE;
    }
}

int
main(int argc, char** argv)
{
    lwm_vehicle_init(&vehicle);
    lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_UDP, 14550);

    struct lwm_action_t get_home_position;
    lwm_action_init(&get_home_position, &vehicle, get_home_position_exec);
    get_home_position.then = get_home_position_then;
    lwm_action_upon_msgid(&get_home_position.then_msgid_list,
        MAVLINK_MSG_ID_COMMAND_ACK, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    lwm_action_submit(&get_home_position, 0);
    lwm_action_poll(&get_home_position);
    if (get_home_position.status == LWM_ACTION_FINISHED)
    {
        struct pos_t* pos = get_home_position.result;
        printf("home position: (%f, %f) %f m\n",
            pos->lat / 1e7, pos->lon / 1e7, pos->alt / 1e3);
    }
    return 0;
}
