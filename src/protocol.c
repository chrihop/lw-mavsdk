#include "lwmavsdk.h"

enum lwm_event_t
{
    LWM_EVENT_MSG,
    LWM_EVENT_IO,
    LWM_EVENT_FAIL,
    LWM_EVENT_TIMEOUT,

    MAX_LWM_EVENT
};

struct lwm_action_t;

struct lwm_action_param_t
{
    struct lwm_action_t* action;
    enum lwm_event_t     event;
};


typedef enum lwm_error_t (*lwm_run_t)(struct lwm_action_t* action, void* data);
typedef void (*lwm_except_t)(
    struct lwm_action_t* action, enum lwm_event_t event, struct lwm_action_param_t* param);
typedef void (*lwm_then_t)(struct lwm_action_t* action, struct lwm_action_param_t* param);
typedef void (*lwm_timeout_t)(struct lwm_action_t* action, struct lwm_action_param_t* param);

#define LWM_MSGID_LIST_SIZE 16

struct lwm_msgid_list_t
{
    uint32_t msgid[LWM_MSGID_LIST_SIZE];
    uint32_t n;
};

struct lwm_action_t
{
    void*                   data;
    struct lwm_vehicle_t*   vehicle;
    lwm_run_t               run;
    struct lwm_msgid_list_t except_msgid_list;
    lwm_except_t            except;
    struct lwm_msgid_list_t then_msgid_list;
    lwm_then_t              then;
    lwm_timeout_t           timeout;
};

struct lwm_action_t*
lwm_action_init(
    struct lwm_action_t* action, struct lwm_vehicle_t* vehicle, lwm_run_t run)
{
    ASSERT(action != NULL);
    ASSERT(vehicle != NULL);

    action->data    = NULL;
    action->vehicle = vehicle;
    action->run     = run;
    action->except  = NULL;
    action->then    = NULL;
    return action;
}


static void
lwm_action_handler(void* context, mavlink_message_t* msg)
{
    ASSERT(context != NULL);
    struct lwm_action_t * action = context;
    struct lwm_action_param_t param;
    param.action = action;


    switch(param->event)
    {
        case LWM_EVENT_MSG:
            ASSERT(msg != NULL);
            /* if the message in the `then` list */
            struct lwm_msgid_list_t * msgid_list = &param->action->then_msgid_list;
            for (size_t i = 0; i < msgid_list->n; i++)
            {
                if (msg->msgid == msgid_list->msgid[i])
                {
                    param->action->then(param->action, param);
                    return;
                }
            }
            /* if the message in the `except` list */
            msgid_list = &param->action->except_msgid_list;
            for (size_t i = 0; i < msgid_list->n; i++)
            {
                if (msg->msgid == msgid_list->msgid[i])
                {
                    param->action->except(param->action, LWM_EVENT_MSG, param);
                    return;
                }
            }
            break;
        case LWM_EVENT_TIMEOUT:
            param->action->timeout(param->action, param);
            break;
        default:
            break;
    }
}

void
lwm_action_submit(struct lwm_action_t* action)
{
    ASSERT(action != NULL);
    ASSERT(action->vehicle != NULL);

    struct lwm_vehicle_t*      vehicle = action->vehicle;
    struct lwm_action_param_t  then_param;
    then_param.action = action;
    then_param.event  = LWM_EVENT_MSG;
    struct lwm_microservice_t * then_service = lwm_microservice_create(vehicle);
    then_service->context = &then_param;




    service->handler = lwm_action_handler;
    for (size_t i = 0; i < action->except_msgid_list.n; i++)
    {
        lwm_microservice_add_to(vehicle, action->except_msgid_list.msgid[i], service);
    }
    for (size_t i = 0; i < action->then_msgid_list.n; i++)
    {
        lwm_microservice_add_to(vehicle, action->then_msgid_list.msgid[i], service);
    }
    enum lwm_error_t err = action->run(action, action->data);


}
