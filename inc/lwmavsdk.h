#ifndef _LWMAVSDK_LWMAVSDK_H_
#define _LWMAVSDK_LWMAVSDK_H_

#include "lwmavsdk.h"
#include "target.h"
#include <v2.0/ardupilotmega/mavlink.h>

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

    MAX_LWM_ERROR
};

/***
 * Connections
 ***/

enum lwm_conn_type_t
{
    LWM_CONN_TYPE_UDP,
    LWM_CONN_TYPE_TCP,
    LWM_CONN_TYPE_SERIAL,
    LWM_CONN_TYPE_CERTIKOS_SERIAL,

    MAX_LWM_CONN_TYPE
};

struct lwm_conn_context_t;

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
            const char* device;
            uint32_t    baudrate;
        } serial;
        struct
        {
            uint32_t device;
            uint32_t baudrate;
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
    struct lwm_conn_context_t conn;
    struct lwm_microservice_registry_t registry;
    struct lwm_service_pool_t          service_pool;
};

#if __cplusplus
extern "C"
{
#endif

enum lwm_error_t lwm_conn_open(struct lwm_conn_context_t* ctx, enum lwm_conn_type_t type, ...);
enum lwm_error_t lwm_conn_send(struct lwm_conn_context_t* ctx, mavlink_message_t* msg);
enum lwm_error_t lwm_conn_recv(struct lwm_conn_context_t* ctx, mavlink_message_t* msg);
void             lwm_conn_close(struct lwm_conn_context_t* ctx);
enum lwm_error_t lwm_conn_register(struct lwm_conn_context_t * ctx, enum lwm_conn_type_t type);

void lwm_microservice_init(struct lwm_vehicle_t * vehicle);
void lwm_microservice_process(struct lwm_vehicle_t * vehicle, mavlink_message_t * msg);
enum lwm_error_t lwm_microservice_add_to(struct lwm_vehicle_t * vehicle, uint32_t msgid, struct lwm_microservice_t * service);
enum lwm_error_t lwm_microservice_remove_from(struct lwm_vehicle_t * vehicle, uint32_t msgid, struct lwm_microservice_t * service);
struct lwm_microservice_t * lwm_microservice_create(struct lwm_vehicle_t * vehicle);
void lwm_microservice_destroy(struct lwm_vehicle_t * vehicle, struct lwm_microservice_t * service);

void lwm_vehicle_init(struct lwm_vehicle_t* vehicle);
enum lwm_error_t lwm_vehicle_spin_once(struct lwm_vehicle_t* vehicle);
void lwm_vehicle_spin(struct lwm_vehicle_t* vehicle);

#if __cplusplus
};
#endif

#endif /* !_LWMAVSDK_LWMAVSDK_H_ */
