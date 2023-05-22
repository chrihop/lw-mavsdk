#include "lwmavsdk.h"

struct lwm_vehicle_t vehicle;

static bool          home_pos_received = false;
struct pos_t
{
    int32_t lat;
    int32_t lon;
    int32_t alt;
};

static struct pos_t  home_pos;

static void
ms_mav_cleanup_home_position(struct lwm_microservice_t* ms)
{
    lwm_microservice_destroy(&vehicle, ms);
}

static void
ms_mav_cmd_home_position(void* context, mavlink_message_t* msg)
{
    ASSERT(context != NULL);
    ASSERT(msg != NULL);

    struct lwm_microservice_t* ms = context;
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t cmd_ack;
        mavlink_msg_command_ack_decode(msg, &cmd_ack);
        if (cmd_ack.command == MAV_CMD_REQUEST_MESSAGE)
        {
            if (cmd_ack.result == MAV_RESULT_ACCEPTED)
            {
                INFO("MAV_CMD_REQUEST_MESSAGE accepted\n");
            }
            else
            {
                WARN("MAV_CMD_REQUEST_MESSAGE rejected\n");
                ms_mav_cleanup_home_position(ms);
            }
        }
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        mavlink_global_position_int_t global_pos;
        mavlink_msg_global_position_int_decode(msg, &global_pos);
        if (global_pos.time_boot_ms > 0)
        {
            INFO("Home position received\n");
            home_pos_received = true;
            global_pos.alt = home_pos.alt;
            global_pos.lat = home_pos.lat;
            global_pos.lon = home_pos.lon;
            ms_mav_cleanup_home_position(ms);
        }
    }
    break;
    default: break;
    }
}

    int main(int argc, char** argv)
    {
        lwm_vehicle_init(&vehicle);
        lwm_conn_open(&vehicle.conn, LWM_CONN_TYPE_UDP, "127.0.0.1", 14550);

        struct lwm_microservice_t* cmd_home_pos
            = lwm_microservice_create(&vehicle);
        cmd_home_pos->handler = ms_mav_cmd_home_position;
        cmd_home_pos->context = cmd_home_pos;
        lwm_microservice_add_to(
            &vehicle, MAVLINK_MSG_ID_COMMAND_ACK, cmd_home_pos);
        lwm_microservice_add_to(
            &vehicle, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, cmd_home_pos);
        enum lwm_error_t err;

        mavlink_command_long_t home_pos_cmd;
        home_pos_cmd.target_system    = 1;
        home_pos_cmd.target_component = 1;
        home_pos_cmd.command          = MAV_CMD_REQUEST_MESSAGE;
        home_pos_cmd.confirmation     = 0;
        home_pos_cmd.param1           = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
        home_pos_cmd.param2           = 0;
        home_pos_cmd.param3           = 0;
        home_pos_cmd.param4           = 0;
        home_pos_cmd.param5           = 0;
        home_pos_cmd.param6           = 0;
        home_pos_cmd.param7           = 0;

        mavlink_message_t msg;
        mavlink_msg_command_long_encode(
            1, 1, &msg, &home_pos_cmd);

        err = LWM_OK;
        while (err == LWM_OK && !home_pos_received)
        {
            err = lwm_vehicle_spin_once(&vehicle);
        }

        printf("Home position: %d %d %d\n", home_pos.lat, home_pos.lon, home_pos.alt);
        lwm_conn_close(&vehicle.conn);

        return 0;
    }
