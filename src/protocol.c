#include "lwmavsdk.h"

void
lwm_action_init(
    struct lwm_action_t* action, struct lwm_vehicle_t* vehicle, lwm_run_t run)
{
    ASSERT(action != NULL);
    ASSERT(vehicle != NULL);

    action->vehicle             = vehicle;
    action->run                 = run;
    action->status              = LWM_ACTION_INIT;
    action->then_msgid_list.n   = 0;
    action->except_msgid_list.n = 0;
    action->timeout_time        = 0;
    action->then                = NULL;
    action->except              = NULL;
    action->timeout             = NULL;
    action->result              = NULL;
}

static void
lwm_do_execute(struct lwm_action_t* action)
{
    enum lwm_error_t err;
    err            = action->run(action, action->data);
    action->status = LWM_ACTION_EXECUTING;
    if (err != LWM_OK)
    {
        if (action->except != NULL)
        {
            struct lwm_action_param_t param;
            param.action          = action;
            param.event           = LWM_EVENT_FAIL;
            param.detail.fail.err = err;
            action->except(action, &param);
        }
        lwm_microservice_destroy(action->vehicle, action->microservice);
        action->status = LWM_ACTION_FAILED;
    }
}

static void
lwm_action_microservice_handler(void* context, mavlink_message_t* msg)
{
    ASSERT(context != NULL);
    ASSERT(msg != NULL);

    struct lwm_action_t*      action = context;
    struct lwm_action_param_t param;
    param.action         = action;
    param.event          = LWM_EVENT_MSG;
    param.detail.msg.msg = msg;

    /* if the message in the `then` list */
    struct lwm_msgid_list_t* msgid_list = &action->then_msgid_list;
    for (size_t i = 0; i < msgid_list->n; i++)
    {
        if (msg->msgid == msgid_list->msgid[i])
        {
            enum lwm_action_continuation_t continuation
                = action->then(action, &param);
            switch (continuation)
            {
            case LWM_ACTION_CONTINUE: break;
            case LWM_ACTION_STOP:
                lwm_microservice_destroy(action->vehicle, action->microservice);
                action->status = LWM_ACTION_FINISHED;
                break;
            case LWM_ACTION_RESTART:
                lwm_do_execute(action);
                action->status = LWM_ACTION_EXECUTING;
                break;
            default: break;
            }
            return;
        }
    }
    /* if the message in the `except` list */
    msgid_list = &action->except_msgid_list;
    for (size_t i = 0; i < msgid_list->n; i++)
    {
        if (msg->msgid == msgid_list->msgid[i])
        {
            action->except(action, &param);
            action->status = LWM_ACTION_FAILED;
            lwm_microservice_destroy(action->vehicle, action->microservice);
            return;
        }
    }
}

static void
lwm_action_timeout_handler(struct lwm_action_t* action, uint64_t time)
{
    ASSERT(action != NULL);

    struct lwm_action_param_t param;
    param.action              = action;
    param.event               = LWM_EVENT_TIMEOUT;
    param.detail.timeout.time = time;
    action->timeout(action, &param);
    action->status = LWM_ACTION_FAILED;
    lwm_microservice_destroy(action->vehicle, action->microservice);
}

void
lwm_action_submit(struct lwm_action_t* action, uint64_t timeout_us)
{
    ASSERT(action != NULL);
    ASSERT(action->vehicle != NULL);

    struct lwm_vehicle_t*      vehicle = action->vehicle;
    struct lwm_microservice_t* action_service
        = lwm_microservice_create(vehicle);
    action_service->context = action;
    action_service->handler = lwm_action_microservice_handler;
    action->microservice    = action_service;
    for (size_t i = 0; i < action->except_msgid_list.n; i++)
    {
        lwm_microservice_add_to(
            vehicle, action->except_msgid_list.msgid[i], action_service);
    }
    for (size_t i = 0; i < action->then_msgid_list.n; i++)
    {
        lwm_microservice_add_to(
            vehicle, action->then_msgid_list.msgid[i], action_service);
    }

    if (timeout_us > 0 && action->timeout != NULL)
    {
        action->timeout_time = time_us() + timeout_us;
    }

    lwm_do_execute(action);
}

enum lwm_error_t
lwm_action_poll_once(struct lwm_action_t* action)
{
    ASSERT(action != NULL);

    if (action->status == LWM_ACTION_FINISHED
        || action->status == LWM_ACTION_FAILED)
    {
        return LWM_ERR_STOPPED;
    }

    if (action->timeout != NULL && time_us() > action->timeout_time)
    {
        lwm_action_timeout_handler(action, action->timeout_time);
        return LWM_ERR_TIMEOUT;
    }

    enum lwm_error_t err = lwm_vehicle_spin_once(action->vehicle);

    return err;
}

enum lwm_error_t
lwm_action_poll(struct lwm_action_t* action)
{
    enum lwm_error_t err;

    if (action->then_msgid_list.n == 0)
    {
        /* not waiting for any response */
        return LWM_OK;
    }

    err = LWM_OK;
    while (err == LWM_OK)
    {
        err = lwm_action_poll_once(action);
    }
    return err;
}

void
lwm_action_upon_msgid(struct lwm_msgid_list_t* list, size_t n, ...)
{
    ASSERT(list != NULL);

    va_list args;
    va_start(args, n);
    for (size_t i = 0; i < n; i++)
    {
        list->msgid[i] = va_arg(args, uint32_t);
    }
    va_end(args);
    list->n = n;
}
