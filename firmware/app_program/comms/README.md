# comms

Owner: Buddy 1, WiFi communication, command and telemetry.

Owns the MQTT session, telemetry and heartbeat publishing, command
subscription and reconnection. API is `include/comms.h`. Knobs are the
`COMMS_*` constants in `common/car_config.h`.

Implemented on lwIP's own MQTT client, added to the kernel's network
library as `lib/libnet/lwip/lwip_utk_mqtt.c` and built with the
`WIFI_MQTT=1` profile. lwIP runs only on the radio owner task, so this
module never calls it: messages cross through two small ring buffers.
Without the profile, `comms_init()` reports the radio absent and the car
drives without telemetry. Received commands are the first letter of the
payload, L, R, S or U in either case, so `{"cmd":"L"}` and `left` both
turn left.

Done means `test_comms.c` passes on the host, and `bench_comms` stays
connected for ten minutes, recovers within 30 seconds after the broker is
restarted, and echoes every command sent from a laptop MQTT client.

Telemetry on `car/telemetry`, one JSON object every 200 ms:

| Key           | Meaning                                              |
|---------------|------------------------------------------------------|
| state         | car_mission_state_t, 0 init to 6 halted, 7 collision |
| speed         | average wheel speed, mm per second                   |
| encl, encr    | encoder counts since boot                            |
| mask          | line sensor bits, 1 left, 2 centre, 4 right          |
| nav           | last car_nav_command_t decoded or received           |
| hump          | peak hump height this run, mm                        |
| dist          | ground distance since boot, mm                       |
| obst.valid    | 1 if the last scan found something                   |
| obst.bearing  | degrees from straight ahead, positive left           |
| obst.range    | closest return, mm                                   |
| obst.width    | estimated width, mm                                  |
| obst.left, obst.right | clearance either side, mm, 0 unknown         |

Heartbeat on `car/heartbeat` every second: `{"uptime_ms":N,"connected":1}`.

| Measurement                    | Value | Notes                        |
|--------------------------------|-------|------------------------------|
| Time to first MQTT connect     |       | from power on, seconds       |
| Reconnect time after drop      |       | pull the broker, time it     |
| Telemetry size per message     | 149   | measured on the broker       |
