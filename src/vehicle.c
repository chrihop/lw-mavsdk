#include "lwmavsdk.h"

void lwm_vehicle_init(struct lwm_vehicle_t* vehicle)
{
    vehicle->conn.status = LWM_CONN_STATUS_CLOSED;
    lwm_microservice_init(vehicle);
}

enum lwm_error_t lwm_vehicle_spin_once(struct lwm_vehicle_t* vehicle)
{
    enum lwm_error_t err;
    mavlink_message_t msg;
    err = lwm_conn_recv(&vehicle->conn, &msg);
    if (err == LWM_OK)
    {
        lwm_microservice_process(vehicle, &msg);
        return LWM_OK;
    }
    else if (err == LWM_ERR_NO_DATA)
    {
        return LWM_OK;
    }
    return err;
}

void lwm_vehicle_spin(struct lwm_vehicle_t* vehicle)
{
    enum lwm_error_t err = LWM_OK;
    while (err == LWM_OK)
    {
        err = lwm_vehicle_spin_once(vehicle);
    }
}
