#include "lwmavsdk.h"
#include <stdio.h>

void
lwm_action_init(
    struct lwm_action_t* action, struct lwm_vehicle_t* vehicle, lwm_run_t run)
{
    ASSERT(action != NULL);
    ASSERT(vehicle != NULL);
    ASSERT(run != NULL);

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
lwm_action_destroy_microservices(struct lwm_action_t* action)
{
    ASSERT(action != NULL);

    struct lwm_vehicle_t *vehicle = action->vehicle;
    for (size_t i = 0; i < action->except_msgid_list.n; i++)
    {
        lwm_microservice_destroy(vehicle,
                action->except_msgid_list.microservice[i]);
        action->except_msgid_list.microservice[i] = NULL;
    }
    for (size_t i = 0; i < action->then_msgid_list.n; i++)
    {
        lwm_microservice_destroy(vehicle,
                action->then_msgid_list.microservice[i]);
        action->then_msgid_list.microservice[i] = NULL;
    }
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
        lwm_action_destroy_microservices(action);
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
    ASSERT(msgid_list->n <= SIZEOF_ARRAY(msgid_list->msgid));

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
                lwm_action_destroy_microservices(action);
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
            lwm_action_destroy_microservices(action);
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
    if(action->timeout)
    {
        action->timeout(action, &param);
    }
    action->status = LWM_ACTION_FAILED;
    lwm_action_destroy_microservices(action);
}

void
lwm_action_submit(struct lwm_action_t* action, uint64_t timeout_us)
{
    ASSERT(action != NULL);
    ASSERT(action->vehicle != NULL);


    if(action->except_msgid_list.n > 0 || action->then_msgid_list.n > 0)
    {
        struct lwm_vehicle_t*      vehicle = action->vehicle;
        for (size_t i = 0; i < action->except_msgid_list.n; i++)
        {
            struct lwm_microservice_t* action_service
                = lwm_microservice_create(vehicle);
            ASSERT(action_service != NULL); /* graceful realloc */

            action_service->context = action;
            action_service->handler = lwm_action_microservice_handler;

            lwm_microservice_add_to(
                vehicle, action->except_msgid_list.msgid[i], action_service);

            action->except_msgid_list.microservice[i] = action_service;
        }
        for (size_t i = 0; i < action->then_msgid_list.n; i++)
        {
            struct lwm_microservice_t* action_service
                = lwm_microservice_create(vehicle);
            ASSERT(action_service != NULL); /* graceful realloc */

            action_service->context = action;
            action_service->handler = lwm_action_microservice_handler;

            lwm_microservice_add_to(
                vehicle, action->then_msgid_list.msgid[i], action_service);

            action->then_msgid_list.microservice[i] = action_service;
        }
    }

    if (timeout_us > 0)
    {
        action->timeout_time = time_us() + timeout_us;
        INFO("timeout_us: %llu (%llu)\n", timeout_us, action->timeout_time);
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

    if (time_us() > action->timeout_time)
    {
        lwm_action_timeout_handler(action, action->timeout_time);
        return LWM_ERR_TIMEOUT;
    }

    enum lwm_error_t err = lwm_vehicle_spin_once(action->vehicle);
    if(err == LWM_ERR_IO)
    {
        action->status = LWM_ACTION_FAILED;
        lwm_action_destroy_microservices(action);
    }

    return err;
}

enum lwm_error_t
lwm_action_poll(struct lwm_action_t* action)
{
    enum lwm_error_t err;

    if (action->then_msgid_list.n == 0)
    {
        /* not waiting for any response */
        printf("not waiting for any response\n");
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
_lwm_action_upon_msgid(struct lwm_msgid_list_t* list, uint32_t *args, size_t n)
{
    ASSERT(list != NULL);
    ASSERT(list->n + n <= SIZEOF_ARRAY(list->msgid));

    for (size_t i = 0; i < n; i++)
    {
        list->msgid[list->n + i] = args[i];
    }
    list->n += n;
}

