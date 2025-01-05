#include "lwmavsdk.h"




enum lwm_action_continuation_t
lwm_command_then_nop(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    mavlink_message_t *msg = param->detail.msg.msg;

    if(msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK)
    {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);

        struct lwm_command_t* x = (struct lwm_command_t*)action->data;

        if(x->msg_id == ack.command)
        {
            if(ack.result != MAV_RESULT_ACCEPTED)
            {
                action->status = LWM_ACTION_FAILED;
            }
            return LWM_ACTION_STOP;
        }
    }
    else
    {
        return LWM_ACTION_STOP;
    }

    return LWM_ACTION_CONTINUE;
}

enum lwm_action_continuation_t
lwm_command_then_nop_free(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    enum lwm_action_continuation_t ret = lwm_command_then_nop(action, param);
    if(ret == LWM_ACTION_STOP)
    {
        free((struct lwm_command_t*)action->data);
    }
    return ret;
}

enum lwm_action_continuation_t
lwm_command_handle_ack(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    ASSERT(action != NULL);
    ASSERT(action->data != NULL);
    ASSERT(param != NULL);

    mavlink_message_t* msg = param->detail.msg.msg;
    ASSERT(msg != NULL);
    ASSERT(msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK);

    //struct lwm_command_t *cmd = (struct lwm_command_t *)action->data;

    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(msg, &ack);

    if (ack.command &&
        ack.result == MAV_RESULT_ACCEPTED)
    {
        action->result = msg;
        return LWM_ACTION_STOP;
    }
    else
    {
    }

    /* ack */
    return LWM_ACTION_CONTINUE;
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
        MAV_CMD_REQUEST_MESSAGE, msgid);
    lwm_action_upon_msgid(&cmd.action.then_msgid_list, msgid);
    lwm_command_execute(&cmd);
    return cmd.action.result;
}

void
lwm_command_request_message_periodic(struct lwm_vehicle_t* vehicle,
    struct lwm_command_t* cmd, uint32_t msgid, uint32_t period_us,
    lwm_then_t callback)
{
    lwm_command_long(vehicle, cmd, callback, MAV_CMD_SET_MESSAGE_INTERVAL,
        msgid, period_us);
    lwm_action_upon_msgid(&cmd->action.then_msgid_list, msgid);
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
    lwm_action_upon_msgid(&cmd.action.then_msgid_list,
        MAVLINK_MSG_ID_HOME_POSITION, MAVLINK_MSG_ID_COMMAND_ACK);
    lwm_command_execute_timeout(&cmd, 1000*1000 /*us*/);
    return cmd.action.result;
}

void
lwm_command_do_set_mode_arducopter(struct lwm_vehicle_t* vehicle,
        enum COPTER_MODE mode)
{
    struct lwm_command_t cmd = {0};
    lwm_command_long(vehicle, &cmd, lwm_command_then_nop, MAV_CMD_DO_SET_MODE,
        (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)mode);
    lwm_action_upon_msgid(&cmd.action.then_msgid_list,
        MAVLINK_MSG_ID_COMMAND_ACK);
    lwm_command_execute_timeout(&cmd, 1000*1000 /*us*/);
}

void
lwm_command_do_set_mode_arducopter_async(struct lwm_vehicle_t* vehicle,
        enum COPTER_MODE mode)
{
    struct lwm_command_t *cmd = calloc(1, sizeof(struct lwm_command_t));
    lwm_command_long(vehicle, cmd, lwm_command_then_nop_free,
            MAV_CMD_DO_SET_MODE,
        (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)mode);
    lwm_command_execute_async(cmd);
}


void
lwm_command_arm_disarm(struct lwm_vehicle_t *vehicle, float arm, float force)
{
    struct lwm_command_t cmd = {0};
        lwm_command_long(vehicle, &cmd, lwm_command_then_nop,
            MAV_CMD_COMPONENT_ARM_DISARM, arm, force);
    lwm_action_upon_msgid(&cmd.action.then_msgid_list,
        MAVLINK_MSG_ID_COMMAND_ACK);
    lwm_command_execute_timeout(&cmd, 1000*1000 /*us*/);
}

void
lwm_command_arm_disarm_async(struct lwm_vehicle_t *vehicle, float arm, float force)
{
    struct lwm_command_t *cmd = calloc(1, sizeof(struct lwm_command_t));
        lwm_command_long(vehicle, cmd, lwm_command_then_nop_free,
            MAV_CMD_COMPONENT_ARM_DISARM, arm, force);
    lwm_command_execute_async(cmd);
}
