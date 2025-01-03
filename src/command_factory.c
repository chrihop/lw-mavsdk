#include "lwmavsdk.h"

enum lwm_action_continuation_t
lwm_command_then_nop(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    return LWM_ACTION_STOP;
}

static enum lwm_action_continuation_t
lwm_command_request_message_then(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    ASSERT(action != NULL);
    ASSERT(action->data != NULL);
    ASSERT(action->vehicle != NULL);
    ASSERT(param != NULL);

    mavlink_message_t* msg = param->detail.msg.msg;
    if (msg->msgid == action->then_msgid_list.msgid[0])
    {
        action->result = msg;
        return LWM_ACTION_STOP;
    }
    return LWM_ACTION_CONTINUE;
}

mavlink_message_t*
lwm_command_request_message(struct lwm_vehicle_t* vehicle, uint32_t msgid)
{
    struct lwm_command_t cmd;
    lwm_command_long(vehicle, &cmd, lwm_command_request_message_then,
        MAV_CMD_REQUEST_MESSAGE, 1, msgid);
    lwm_action_upon_msgid(&cmd.action.then_msgid_list, 1, msgid);
    lwm_command_execute(&cmd);
    return cmd.action.result;
}

void
lwm_command_request_message_periodic(struct lwm_vehicle_t* vehicle,
    struct lwm_command_t* cmd, uint32_t msgid, uint32_t period_us,
    lwm_then_t callback)
{
    lwm_command_long(vehicle, cmd, callback, MAV_CMD_SET_MESSAGE_INTERVAL, 2,
        msgid, period_us);
    lwm_action_upon_msgid(&cmd->action.then_msgid_list, 1, msgid);
    lwm_command_execute_async(cmd);
}

static enum lwm_action_continuation_t
lwm_command_get_home_position_then(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    ASSERT(action != NULL);
    ASSERT(action->data != NULL);
    ASSERT(action->vehicle != NULL);
    ASSERT(param != NULL);

    mavlink_message_t* msg = param->detail.msg.msg;
    if (msg->msgid == MAVLINK_MSG_ID_HOME_POSITION)
    {
        action->result = msg;
        return LWM_ACTION_STOP;
    }

    /* ack */
    return LWM_ACTION_CONTINUE;
}

mavlink_message_t*
lwm_command_get_home_position(struct lwm_vehicle_t* vehicle)
{
    struct lwm_command_t cmd = {0};
    lwm_command_long(vehicle, &cmd, lwm_command_get_home_position_then,
        MAV_CMD_GET_HOME_POSITION, 0);
    lwm_action_upon_msgid(&cmd.action.then_msgid_list, 2,
        MAVLINK_MSG_ID_HOME_POSITION, MAVLINK_MSG_ID_COMMAND_ACK);
    lwm_command_execute_timeout(&cmd, 1000*1000 /*us*/);
    return cmd.action.result;
}

void
lwm_command_do_set_mode(struct lwm_vehicle_t* vehicle,
        uint32_t mode,
        uint32_t custom_mode,
        uint32_t custom_sub_mode)
{
    struct lwm_command_t cmd;
    lwm_command_long(vehicle, &cmd, lwm_command_then_nop, MAV_CMD_DO_SET_MODE,
        3, mode, custom_mode, custom_sub_mode);
    lwm_command_execute_async(&cmd);
}

void
lwm_command_arm_disarm(struct lwm_vehicle_t *vehicle, uint32_t arm, uint32_t force)
{
    struct lwm_command_t cmd;
    lwm_command_long(vehicle, &cmd, lwm_command_then_nop,
            MAV_CMD_COMPONENT_ARM_DISARM, 2, arm, force);
    lwm_command_execute_async(&cmd);
}
