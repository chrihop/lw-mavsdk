#include "lwmavsdk.h"
#include <math.h>
#include <string.h>

static mavlink_global_position_int_t current_position;
static mavlink_battery_status_t      current_battery_status;
static int                           started = 0;

double
get_distance_meters(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{

    double dlat  = lat1 - lat2;
    double dlong = lon1 - lon2;

    double dist = sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5;
    return dist / 10000000.0;
}

static void
callback_on_current_position(void* context, mavlink_message_t* msg)
{
    ASSERT(msg != NULL);
    if (msg->msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
    {
        return;
    }

    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(msg, &global_pos);

    memcpy(
        &current_position, &global_pos, sizeof(mavlink_global_position_int_t));
    started = started | 1;

    // INFO("Current Position: (%f, %f) at %f m\n",
    //     global_pos.lat / 10000000.0,
    //     global_pos.lon / 10000000.0,
    //     global_pos.alt / 1000.0);
}

static void
callback_on_battery_status(void* context, mavlink_message_t* msg)
{
    ASSERT(msg != NULL);

    if (msg->msgid != MAVLINK_MSG_ID_BATTERY_STATUS)
    {
        return;
    }

    mavlink_battery_status_t battery_status;
    mavlink_msg_battery_status_decode(msg, &battery_status);

    memcpy(&current_battery_status, &battery_status,
        sizeof(mavlink_battery_status_t));

    started = started | 2;

    // INFO("Battery Status: %d%%\n", battery_status.battery_remaining);
}

void
battery_fence(struct lwm_vehicle_t* vehicle)
{
    enum lwm_error_t   err = LWM_OK;
    mavlink_message_t* msg;
    msg = lwm_command_get_home_position(vehicle);
    if (msg == NULL)
    {
        PANIC("Failed to get home position\n");
    }
    mavlink_home_position_t home_position;
    mavlink_msg_home_position_decode(msg, &home_position);
    INFO("Home Position: (%f, %f) at %f m\n",
        home_position.latitude / 10000000.0,
        home_position.longitude / 10000000.0, home_position.altitude / 1000.0);

    struct lwm_microservice_t* curr_pos = lwm_microservice_create(vehicle);
    curr_pos->handler                   = callback_on_current_position;
    curr_pos->context                   = NULL;
    lwm_microservice_add_to(
        vehicle, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, curr_pos);

    struct lwm_microservice_t* battery = lwm_microservice_create(vehicle);
    battery->handler                   = callback_on_battery_status;
    battery->context                   = NULL;
    lwm_microservice_add_to(vehicle, MAVLINK_MSG_ID_BATTERY_STATUS, battery);

    while (err == LWM_OK && started != 3)
        err = lwm_vehicle_spin_once(vehicle);

    mavlink_global_position_int_t prev_position;
    double                        distance_traveled = 0;
    double initial_remaining_level = current_battery_status.battery_remaining;
    double avg_battery_consumption = 0;

    memcpy(&prev_position, &current_position,
        sizeof(mavlink_global_position_int_t));

    uint64_t ts             = time_us() + 1000000;
    bool     return_started = false;
    err                     = LWM_OK;
    while (err == LWM_OK)
    {
        /* message show frequency 1 Hz */
        if (ts < time_us())
        {
            ts = time_us() + 1000000;

            double delta_distance = get_distance_meters(current_position.lat,
                current_position.lon, prev_position.lat, prev_position.lon);

            distance_traveled = distance_traveled + delta_distance;

            double distance_to_home = get_distance_meters(current_position.lat,
                current_position.lon, home_position.latitude,
                home_position.longitude);

            INFO("--------------------------------------------------\n");
            INFO("delta_distance = %f m, distance_traveled = %f m, "
                 "distance_to_home = %f m\n",
                delta_distance, distance_traveled, distance_to_home);

            // INFO("Current Position: (%f, %f) at %f m\n",
            //     current_position.lat / 10000000.0,
            //     current_position.lon / 10000000.0,
            //     current_position.alt / 1000.0);

            int battery_remaining = current_battery_status.battery_remaining;
            int battery_remaining_with_buffer = battery_remaining
                - 7; // so, underestimate the current battery level

            if (battery_remaining_with_buffer < 0)
                battery_remaining_with_buffer = 0;

            double battery_consumed
                = initial_remaining_level - battery_remaining_with_buffer;

            if (distance_traveled > 300 && battery_consumed > 0)
            {
                avg_battery_consumption = distance_traveled / battery_consumed;
                INFO("Battery remaining: %d%% (- buffer = %d%%), "
                     "avg_battery_consumption: %f meters/%%\n",
                    battery_remaining, battery_remaining_with_buffer,
                    avg_battery_consumption);

                double remaining_range
                    = battery_remaining_with_buffer * avg_battery_consumption;
                INFO(
                    "Remaining range: %f meters, distance to home: %f meters\n",
                    remaining_range, distance_to_home);

                if (return_started)
                {
                    INFO("\t!!! RETURNING TO HOME!!!\n");
                }
                else
                {
                    if (remaining_range < distance_to_home)
                    {
                        INFO("\t!!! NEED TO RETURN TO HOME!!!\n");
                        lwm_command_do_set_mode(vehicle, COPTER_MODE_RTL);
                        return_started = true;
                    }
                }
            }

            memcpy(&prev_position, &current_position,
                sizeof(mavlink_global_position_int_t));
        }

        err = lwm_vehicle_spin_once(vehicle);
    }
}
