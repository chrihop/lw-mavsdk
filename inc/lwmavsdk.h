#ifndef _LWMAVSDK_LWMAVSDK_H_
#define _LWMAVSDK_LWMAVSDK_H_

#include "lwmavsdk.h"
#include "target.h"
#include <v2.0/ardupilotmega/mavlink.h>

#define SYSTEM_ID    127
#define COMPONENT_ID 1

enum lwm_error_t
{
    LWM_OK,
    LWM_ERR_BAD_MESSAGE,
    LWM_ERR_BAD_CONNECTION,
    LWM_ERR_BAD_PARAM,
    LWM_ERR_IO,
    LWM_ERR_NO_DATA,
    LWM_ERR_NO_MEM,
    LWM_ERR_NOT_SUPPORTED,
    LWM_ERR_TIMEOUT,
    LWM_ERR_STOPPED,

    MAX_LWM_ERROR
};

/***
 * Connections
 ***/

enum lwm_conn_type_t
{
    LWM_CONN_TYPE_UDP,
    LWM_CONN_TYPE_UDP_CLIENT,
    LWM_CONN_TYPE_TCP,
    LWM_CONN_TYPE_SERIAL,
    LWM_CONN_TYPE_CERTIKOS_SERIAL,

    MAX_LWM_CONN_TYPE
};

struct lwm_conn_context_t;

enum lwm_serial_baudrate_t
{
    LWM_SERIAL_BAUDRATE_9600,
    LWM_SERIAL_BAUDRATE_19200,
    LWM_SERIAL_BAUDRATE_38400,
    LWM_SERIAL_BAUDRATE_57600,
    LWM_SERIAL_BAUDRATE_115200,
    LWM_SERIAL_BAUDRATE_230400,
    LWM_SERIAL_BAUDRATE_460800,
    LWM_SERIAL_BAUDRATE_921600,

    MAX_LWM_SERIAL_BAUDRATE
};

struct lwm_conn_params_t
{
    enum lwm_conn_type_t type;
    union
    {
        struct
        {
            const char* host;
            uint16_t    port;
        } udp;
        struct
        {
            const char* host;
            uint16_t    port;
        } tcp;
        struct
        {
            const char*                device;
            enum lwm_serial_baudrate_t baudrate;
        } serial;
        struct
        {
            uint32_t device;
        } certikos_serial;
    } params;
};

typedef enum lwm_error_t (*lwm_conn_open_t)(
    struct lwm_conn_context_t* ctx, struct lwm_conn_params_t* params);
typedef enum lwm_error_t (*lwm_conn_send_t)(
    struct lwm_conn_context_t* ctx, const uint8_t* buf, size_t len);
typedef ssize_t (*lwm_conn_recv_t)(
    struct lwm_conn_context_t* ctx, uint8_t* buf, size_t len);
typedef void (*lwm_conn_close_t)(struct lwm_conn_context_t* ctx);

#define LWM_READ_BUFFER_SIZE 512

struct lwm_read_buffer_t
{
    uint8_t buffer[LWM_READ_BUFFER_SIZE];
    size_t  len;
    size_t  pos;
};

enum lwm_conn_status_t
{
    LWM_CONN_STATUS_CLOSED,
    LWM_CONN_STATUS_OPEN,
    LWM_CONN_STATUS_ERROR,
    LWM_CONN_STATUS_UNKNOWN
};

struct lwm_conn_context_t
{
    enum lwm_conn_status_t   status;
    enum lwm_conn_type_t     type;
    void*                    opaque;
    lwm_conn_open_t          open;
    lwm_conn_send_t          send;
    lwm_conn_recv_t          recv;
    lwm_conn_close_t         close;
    uint8_t                  output[MAVLINK_MAX_PACKET_LEN];
    struct lwm_read_buffer_t input;
    mavlink_status_t         rx_status;
    mavlink_message_t        rx_message;
};

/***
 * Microservices
 ***/
struct lwm_microservice_t;

struct lwm_microservice_t
{
    bool  is_active;
    void* context;
    void (*handler)(void* context, mavlink_message_t* msg);
    struct lwm_microservice_t* next;
    struct lwm_microservice_t* prev;
};

#define MAX_LWM_SERVICE 128

struct lwm_service_pool_t
{
    struct lwm_microservice_t services[MAX_LWM_SERVICE];
    uint32_t                  n;
};

struct lwm_service_list_t
{
    struct lwm_microservice_t* head;
    struct lwm_microservice_t* tail;
};

struct lwm_microservice_registry_entry_t
{
    bool                      is_active;
    uint32_t                  msgid;
    struct lwm_service_list_t list;
};

#define MAX_LWM_SERVICE_REGISTRY 128

struct lwm_microservice_registry_t
{
    struct lwm_microservice_registry_entry_t entries[MAX_LWM_SERVICE_REGISTRY];
    uint32_t                                 n;
};

/***
 * Vehicle
 ***/
struct lwm_vehicle_t
{
    struct lwm_conn_context_t          conn;
    struct lwm_microservice_registry_t registry;
    struct lwm_service_pool_t          service_pool;
    uint32_t                           sysid;
    uint32_t                           compid;
};

/***
 * Protocol
 ***/

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
    union
    {
        struct
        {
            mavlink_message_t* msg;
        } msg;
        struct
        {
            enum lwm_error_t err;
        } fail;
        struct
        {
            uint64_t time;
        } timeout;
    } detail;
};

enum lwm_action_continuation_t
{
    LWM_ACTION_CONTINUE,
    LWM_ACTION_STOP,
    LWM_ACTION_RESTART
};

typedef enum lwm_error_t (*lwm_run_t)(struct lwm_action_t* action, void* data);
typedef void (*lwm_except_t)(
    struct lwm_action_t* action, struct lwm_action_param_t* param);
typedef enum lwm_action_continuation_t (*lwm_then_t)(
    struct lwm_action_t* action, struct lwm_action_param_t* param);
typedef void (*lwm_timeout_t)(
    struct lwm_action_t* action, struct lwm_action_param_t* param);

#define LWM_MSGID_LIST_SIZE 16

struct lwm_msgid_list_t
{
    uint32_t n;
    uint32_t msgid[LWM_MSGID_LIST_SIZE];
};

enum lwm_action_status_t
{
    LWM_ACTION_INIT,
    LWM_ACTION_EXECUTING,
    LWM_ACTION_FINISHED,
    LWM_ACTION_FAILED,

    MAX_LWM_ACTION_STATUS
};

struct lwm_action_t
{
    enum lwm_action_status_t   status;
    void*                      data;
    void*                      result;
    struct lwm_vehicle_t*      vehicle;
    struct lwm_microservice_t* microservice;
    lwm_run_t                  run;
    struct lwm_msgid_list_t    except_msgid_list;
    lwm_except_t               except;
    struct lwm_msgid_list_t    then_msgid_list;
    lwm_then_t                 then;
    uint64_t                   timeout_time;
    lwm_timeout_t              timeout;
};

/***
 * Command
 ***/
enum lwm_command_type_t
{
    LWM_COMMAND_TYPE_INT,
    LWM_COMMAND_TYPE_LONG,
};

struct lwm_command_t
{
    struct lwm_action_t     action;
    enum lwm_command_type_t type;
    union
    {
        mavlink_command_int_t  _int;
        mavlink_command_long_t _long;
    } cmd;
    mavlink_message_t out_msg;
    lwm_then_t        do_then;
};

#if __cplusplus
extern "C"
{
#endif

    enum lwm_error_t lwm_conn_open(
        struct lwm_conn_context_t* ctx, enum lwm_conn_type_t type, ...);
    enum lwm_error_t lwm_conn_send(
        struct lwm_conn_context_t* ctx, mavlink_message_t* msg);
    enum lwm_error_t lwm_conn_recv(
        struct lwm_conn_context_t* ctx, mavlink_message_t* msg);
    void             lwm_conn_close(struct lwm_conn_context_t* ctx);
    enum lwm_error_t lwm_conn_register(
        struct lwm_conn_context_t* ctx, enum lwm_conn_type_t type);

    void lwm_microservice_init(struct lwm_vehicle_t* vehicle);
    void lwm_microservice_process(
        struct lwm_vehicle_t* vehicle, mavlink_message_t* msg);
    enum lwm_error_t lwm_microservice_add_to(struct lwm_vehicle_t* vehicle,
        uint32_t msgid, struct lwm_microservice_t* service);
    enum lwm_error_t lwm_microservice_remove_from(struct lwm_vehicle_t* vehicle,
        uint32_t msgid, struct lwm_microservice_t* service);
    struct lwm_microservice_t* lwm_microservice_create(
        struct lwm_vehicle_t* vehicle);
    void lwm_microservice_destroy(
        struct lwm_vehicle_t* vehicle, struct lwm_microservice_t* service);

    void             lwm_vehicle_init(struct lwm_vehicle_t* vehicle);
    enum lwm_error_t lwm_vehicle_spin_once(struct lwm_vehicle_t* vehicle);
    void             lwm_vehicle_spin(struct lwm_vehicle_t* vehicle);

    void             lwm_action_init(struct lwm_action_t* action,
                    struct lwm_vehicle_t* vehicle, lwm_run_t run);
    enum lwm_error_t lwm_action_poll_once(struct lwm_action_t* action);
    void lwm_action_submit(struct lwm_action_t* action, uint64_t timeout_us);
    void lwm_action_upon_msgid(struct lwm_msgid_list_t* list, size_t n, ...);
    enum lwm_error_t lwm_action_poll(struct lwm_action_t* action);

    void             lwm_command_long(struct lwm_vehicle_t* vehicle,
                    struct lwm_command_t* x, lwm_then_t then, uint16_t command, size_t argc,
                    ...);
    void lwm_command_int(struct lwm_vehicle_t* vehicle, struct lwm_command_t* x,
        lwm_then_t then, uint16_t command, size_t argc, ...);
    void lwm_command_execute(struct lwm_command_t* x);
    void lwm_command_execute_timeout(
        struct lwm_command_t* x, uint64_t timeout_us);
    void               lwm_command_execute_async(struct lwm_command_t* x);

    mavlink_message_t* lwm_command_request_message(
        struct lwm_vehicle_t* vehicle, uint32_t msgid);
    /**
     * @warning calling this function may lead Ardupilot to crash
     */
    void lwm_command_request_message_periodic(struct lwm_vehicle_t* vehicle,
        struct lwm_command_t* cmd, uint32_t msgid, uint32_t period_us,
        lwm_then_t callback);
    mavlink_message_t* lwm_command_get_home_position(
        struct lwm_vehicle_t* vehicle);
    void lwm_command_do_set_mode(
        struct lwm_vehicle_t* vehicle, uint32_t custom_mode);

#if __cplusplus
};
#endif

#endif /* !_LWMAVSDK_LWMAVSDK_H_ */
