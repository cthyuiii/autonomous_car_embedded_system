/** @file test_comms.c
 *
 * @brief Host contract test for the comms module. No hardware needed.
 *
 * NOTE: The host build has no radio. The transport is faked as always
 * connected, and received payloads are injected through the host hook so
 * the command parser and the handler dispatch are exercised for real.
 *
 * Owner: Buddy 1, WiFi communication, command and telemetry. Add an assert for
 * every new guarantee and never delete one to make it pass.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "comms.h"

static car_nav_command_t g_received = CAR_NAV_NONE;
static uint32_t          g_calls    = 0u;

static void handle_command (car_nav_command_t command);

int main (void)
{
    car_telemetry_t telemetry = { 0 };

    assert(false == comms_is_connected());
    assert(CAR_OK == comms_set_command_handler(handle_command));
    assert(CAR_OK == comms_init());
    assert(CAR_OK == comms_poll());
    assert(CAR_OK == comms_publish_telemetry(&telemetry));
    assert(CAR_ERR_RANGE == comms_publish_telemetry(NULL));
    assert(CAR_OK == comms_publish_heartbeat());
    assert(true == comms_is_connected());

    /* A plain word, a JSON wrapped letter, and noise. */
    comms_host_inject("left", 4u);
    assert(CAR_OK == comms_poll());
    assert(1u == g_calls);
    assert(CAR_NAV_LEFT == g_received);

    comms_host_inject("{\"cmd\":\"u\"}", 11u);
    assert(CAR_OK == comms_poll());
    assert(2u == g_calls);
    assert(CAR_NAV_UTURN == g_received);

    comms_host_inject("123", 3u);
    assert(CAR_OK == comms_poll());
    assert(2u == g_calls);

    /* A full snapshot, every field at its widest and the longest action
     * name, still fits the payload buffer, so it fits an MQTT slot. */
    telemetry.mission_state                = CAR_STATE_COLLISION;
    telemetry.action                       = CAR_ACTION_BACKING_OFF;
    telemetry.speed_mm_per_sec             = 65535u;
    telemetry.line_sensor_mask             = 255u;
    telemetry.last_nav_command             = CAR_NAV_UTURN;
    telemetry.peak_hump_height_mm          = 65535u;
    telemetry.pitch_deg                    = -32768;
    telemetry.heading_deg                  = -32768;
    telemetry.turn_rate_dps                = -32768;
    telemetry.motion_event                 = CAR_MOTION_DESCENDING;
    telemetry.terrain_rough_milli_g        = 65535u;
    telemetry.accel_milli_g                = 65535u;
    telemetry.hump_count                   = 65535u;
    telemetry.last_hump_height_mm          = 65535u;
    telemetry.encoder_count_left           = 4294967295u;
    telemetry.encoder_count_right          = 4294967295u;
    telemetry.total_distance_mm            = 4294967295u;
    telemetry.last_obstacle.b_is_valid     = true;
    telemetry.last_obstacle.bearing_deg    = -180;
    telemetry.last_obstacle.closest_range_mm   = 65535u;
    telemetry.last_obstacle.width_mm           = 65535u;
    telemetry.last_obstacle.clearance_left_mm  = 65535u;
    telemetry.last_obstacle.clearance_right_mm = 65535u;
    assert(CAR_OK == comms_publish_telemetry(&telemetry));
    assert(CAR_OK == comms_publish_terrain(&telemetry));
    assert(CAR_ERR_RANGE == comms_publish_terrain(NULL));

    return 0;
}

static void handle_command (car_nav_command_t command)
{
    g_received = command;
    g_calls++;
}

/*** end of file ***/
