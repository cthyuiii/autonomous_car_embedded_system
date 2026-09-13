/** @file test_comms.c
 *
 * @brief Host contract test for the comms module. No hardware needed.
 *
 * NOTE: A real implementation needs the CYW43 driver, so once implemented
 * this test needs a stub header for it. The contract stays the same.
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>

#include "comms.h"

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

    return 0;
}

static void handle_command (car_nav_command_t command)
{
    (void)command;
}

/*** end of file ***/

