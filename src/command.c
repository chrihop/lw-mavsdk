#include "lwmavsdk.h"

static enum lwm_error_t
lwm_command_exec(struct lwm_action_t* action, void* data)
{
    ASSERT(action != NULL);
    ASSERT(action->vehicle != NULL);
    ASSERT(data != NULL);

    struct lwm_command_t* x = (struct lwm_command_t*)data;
    struct lwm_vehicle_t* vehicle = x->action.vehicle;

    enum lwm_error_t err = lwm_conn_send(&vehicle->conn, &x->out_msg);
    if (err != LWM_OK)
    {
        WARN("command %d send failed: %d\n", x->msg_id, err);
    }
    return err;
}



void
_lwm_command_long(struct lwm_vehicle_t* vehicle, struct lwm_command_t* x,
    lwm_then_t then, MAV_CMD command, float params[7])
{
    ASSERT(x != NULL);
    ASSERT(then != NULL);

    x->msg_id = MAVLINK_MSG_ID_COMMAND_LONG;
    mavlink_msg_command_long_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &x->out_msg,
        vehicle->sysid,
        vehicle->compid,
        command,
        0, /* confirmation */
        params[0],
        params[1],
        params[2],
        params[3],
        params[4],
        params[5],
        params[6]);

    lwm_action_init(&x->action, vehicle, lwm_command_exec);
    x->action.data = x;
    x->action.then = then;
}



void
_lwm_command_int(
        struct lwm_vehicle_t* vehicle,
        struct lwm_command_t* cmd,
        lwm_then_t then,
        MAV_FRAME frame,
        MAV_CMD command,
        uint8_t current,
        uint8_t autocontinue,
        float param1,
        float param2,
        float param3,
        float param4,
        int32_t x,
        int32_t y,
        float z)
{
    ASSERT(cmd != NULL);
    ASSERT(then != NULL);

    cmd->msg_id = MAVLINK_MSG_ID_COMMAND_INT;
    mavlink_msg_command_int_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &cmd->out_msg,
        vehicle->sysid,
        vehicle->compid,
        frame,
        command,
        current,
        autocontinue,
        param1,
        param2,
        param3,
        param4,
        x,
        y,
        z);

    lwm_action_init(&cmd->action, vehicle, lwm_command_exec);
    cmd->action.data = cmd;
    cmd->action.then = then;
}


void
lwm_command_mission_item_int(struct lwm_vehicle_t* vehicle, struct lwm_command_t* x,
    lwm_then_t then, mavlink_mission_item_int_t* item)
{
    ASSERT(x != NULL);
    ASSERT(then != NULL);

    x->msg_id = MAVLINK_MSG_ID_MISSION_ITEM_INT;
    mavlink_msg_mission_item_int_encode(
        SYSTEM_ID, COMPONENT_ID, &x->out_msg, item);

    lwm_action_init(&x->action, vehicle, lwm_command_exec);
    x->action.data = x;
    x->action.then = then;
}


static enum lwm_action_continuation_t
lwm_command_mission_list_callback(
        struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    struct lwm_command_t* x = (struct lwm_command_t*)action->data;
    mavlink_message_t* msg = param->detail.msg.msg;

    if (msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT ||
        msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST)
    {
        uint32_t seq;

        if(msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT)
        {
            mavlink_mission_request_int_t dec;
            mavlink_msg_mission_request_int_decode(msg, &dec);
            seq = dec.seq;
        }
        else
        {
            mavlink_mission_request_t dec;
            mavlink_msg_mission_request_decode(msg, &dec);
            seq = dec.seq;
        }

        if(seq >= x->mission.items_size)
        {
            WARN("Mission request out of range: %d\n", seq);
            return LWM_ACTION_STOP;
        }

        x->mission.items[seq].seq = seq;
        mavlink_msg_mission_item_int_encode(
            SYSTEM_ID, COMPONENT_ID, &x->out_msg, &x->mission.items[seq]);
        lwm_conn_send(&action->vehicle->conn, &x->out_msg);
    }
    else if (msg->msgid == MAVLINK_MSG_ID_MISSION_ACK)
    {
        return x->mission.finally(action, param);
    }

    return LWM_ACTION_CONTINUE;
}


void
lwm_command_mission_clear_all(
        struct lwm_vehicle_t* vehicle,
        struct lwm_command_t* x,
        lwm_then_t then,
        MAV_MISSION_TYPE type)
{
    ASSERT(x != NULL);
    ASSERT(then != NULL);

    memset(x, 0, sizeof(*x));
    x->msg_id = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
    mavlink_msg_mission_clear_all_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &x->out_msg,
        vehicle->sysid,
        vehicle->compid,
        type);

    lwm_action_init(&x->action, vehicle, lwm_command_exec);
    x->action.data = x;
    x->action.then = then;
    lwm_action_upon_msgid(&x->action.then_msgid_list,
            MAVLINK_MSG_ID_MISSION_ACK);
}




void
lwm_command_mission_list(
        struct lwm_vehicle_t* vehicle,
        struct lwm_command_t* x,
        lwm_then_t then,
        MAV_MISSION_TYPE type,
        mavlink_mission_item_int_t* items,
        size_t items_size)
{
    ASSERT(x != NULL);
    ASSERT(then != NULL);

    memset(x, 0, sizeof(*x));

    x->msg_id = MAVLINK_MSG_ID_MISSION_COUNT;
    mavlink_msg_mission_count_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &x->out_msg,
        vehicle->sysid,
        vehicle->compid,
        items_size,
        type,
        0 /* opaque id */);

    x->mission.items = items;
    x->mission.items_size = items_size;
    x->mission.finally = then;

    lwm_action_init(&x->action, vehicle, lwm_command_exec);
    x->action.data = x;
    x->action.then = lwm_command_mission_list_callback;
    lwm_action_upon_msgid(&x->action.then_msgid_list,
            MAVLINK_MSG_ID_MISSION_REQUEST_INT,
            MAVLINK_MSG_ID_MISSION_REQUEST,
            MAVLINK_MSG_ID_MISSION_ACK);
}


void
lwm_command_execute(struct lwm_command_t* x)
{
    ASSERT(x != NULL);
    ASSERT(x->action.vehicle != NULL);

    lwm_action_submit(&x->action, 0);
    lwm_action_poll(&x->action);
}

void
lwm_command_execute_timeout(struct lwm_command_t* x, uint64_t timeout_us)
{
    ASSERT(x != NULL);
    ASSERT(x->action.vehicle != NULL);

    lwm_action_submit(&x->action, timeout_us);
    lwm_action_poll(&x->action);
}

void
lwm_command_execute_async(struct lwm_command_t* x)
{
    ASSERT(x != NULL);
    ASSERT(x->action.vehicle != NULL);

    lwm_action_submit(&x->action, 0);
}
