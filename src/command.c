#include "lwmavsdk.h"

static enum lwm_error_t
lwm_command_exec(struct lwm_action_t* action, void* data)
{
    ASSERT(action != NULL);
    ASSERT(action->data != NULL);
    ASSERT(action->vehicle != NULL);
    ASSERT(data != NULL);

    struct lwm_command_t* x = (struct lwm_command_t*)data;

    switch (x->type)
    {
    case LWM_COMMAND_TYPE_INT:
        mavlink_msg_command_int_encode(
            SYSTEM_ID, COMPONENT_ID, &x->out_msg, &x->cmd._int);
        break;
    case LWM_COMMAND_TYPE_LONG:
        mavlink_msg_command_long_encode(
            SYSTEM_ID, COMPONENT_ID, &x->out_msg, &x->cmd._long);
        break;
    default:
        WARN("Unknown command type: %d\n", x->type);
        return LWM_ERR_BAD_PARAM;
    }

    struct lwm_vehicle_t* vehicle = x->action.vehicle;
    enum lwm_error_t      err     = lwm_conn_send(&vehicle->conn, &x->out_msg);
    if (err != LWM_OK)
    {
        WARN("command %d send failed: %d\n", x->cmd._long.command, err);
    }
    return err;
}

static enum lwm_action_continuation_t
lwm_command_then_ack(
    struct lwm_action_t* action, struct lwm_action_param_t* param)
{
    ASSERT(action != NULL);
    ASSERT(action->data != NULL);
    ASSERT(action->vehicle != NULL);
    ASSERT(param != NULL);

    struct lwm_command_t* x   = (struct lwm_command_t*)action->data;
    mavlink_message_t*    msg = param->detail.msg.msg;
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK)
    {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        if (ack.command == x->cmd._long.command
            && ack.result != MAV_RESULT_ACCEPTED)
        {
            WARN("Command failed: %d\n", ack.result);
//            action->result = NULL;
            return LWM_ACTION_CONTINUE;
        }
        if (ack.command == x->cmd._long.command
            && ack.result == MAV_RESULT_ACCEPTED)
        {
            INFO("Command %d accepted\n", ack.command);
        }
    }
    else
    {
        return x->do_then(action, param);
    }

    return LWM_ACTION_CONTINUE;
}

void
lwm_command_long(struct lwm_vehicle_t* vehicle, struct lwm_command_t* x,
    lwm_then_t then, uint16_t command, size_t argc, ...)
{
    ASSERT(x != NULL);
    ASSERT(then != NULL);
    ASSERT(argc < 8);

    x->type = LWM_COMMAND_TYPE_LONG;
    memset(&x->cmd._long, 0, sizeof(x->cmd._long));
    x->cmd._long.command          = command;
    x->cmd._long.target_system    = vehicle->sysid;
    x->cmd._long.target_component = vehicle->compid;
    x->cmd._long.confirmation     = 1;
    size_t     n;
    va_list ap;
    va_start(ap, argc);
    for (n = 0; n < argc; ++n)
    {
        switch (n)
        {
        case 0: x->cmd._long.param1 = va_arg(ap, size_t); break;
        case 1: x->cmd._long.param2 = va_arg(ap, size_t); break;
        case 2: x->cmd._long.param3 = va_arg(ap, size_t); break;
        case 3: x->cmd._long.param4 = va_arg(ap, size_t); break;
        case 4: x->cmd._long.param5 = va_arg(ap, size_t); break;
        case 5: x->cmd._long.param6 = va_arg(ap, size_t); break;
        case 6: x->cmd._long.param7 = va_arg(ap, size_t); break;
        default: WARN("Too many arguments for command: %lu\n", argc); break;
        }
    }
    va_end(ap);

    lwm_action_init(&x->action, vehicle, lwm_command_exec);
    x->do_then     = then;
    x->action.data = x;
    x->action.then = lwm_command_then_ack;
}

void
lwm_command_int(struct lwm_vehicle_t* vehicle, struct lwm_command_t* x,
    lwm_then_t then, uint16_t command, size_t argc, ...)
{
    ASSERT(x != NULL);
    ASSERT(then != NULL);
    ASSERT(argc < 8);

    x->type                      = LWM_COMMAND_TYPE_INT;
    memset(&x->cmd._long, 0, sizeof(x->cmd._long));
    x->cmd._int.command          = command;
    x->cmd._int.target_system    = vehicle->sysid;
    x->cmd._int.target_component = vehicle->compid;
    size_t     n;
    va_list ap;
    va_start(ap, argc);
    for (n = 0; n < argc; ++n)
    {
        switch (n)
        {
        case 0: x->cmd._int.param1 = va_arg(ap, size_t); break;
        case 1: x->cmd._int.param2 = va_arg(ap, size_t); break;
        case 2: x->cmd._int.param3 = va_arg(ap, size_t); break;
        case 3: x->cmd._int.param4 = va_arg(ap, size_t); break;
        case 4: x->cmd._int.x = va_arg(ap, size_t); break;
        case 5: x->cmd._int.y = va_arg(ap, size_t); break;
        case 6: x->cmd._int.z = va_arg(ap, size_t); break;
        default: WARN("Too many arguments for command: %lu\n", argc); break;
        }
    }
    va_end(ap);

    lwm_action_init(&x->action, vehicle, lwm_command_exec);
    x->do_then     = then;
    x->action.data = x;
    x->action.then = lwm_command_then_ack;
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
