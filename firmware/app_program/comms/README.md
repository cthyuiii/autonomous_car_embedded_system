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
| action        | what the car is doing now, as text, table below      |
| speed         | average wheel speed, mm per second                   |
| encl, encr    | encoder counts since boot                            |
| mask          | line sensor bits, 1 left, 2 barcode, 4 right         |
| nav           | last car_nav_command_t decoded or received           |
| barcode       | last barcode letter read, "" before the first        |
| barcodes      | barcodes read since boot, any letter                 |
| hump          | peak hump height this run, mm                        |
| dist          | ground distance since boot, mm                       |
| obst.valid    | 1 if the last scan found something                   |
| obst.bearing  | degrees from straight ahead, positive left           |
| obst.range    | closest return, mm                                   |
| obst.width    | estimated width, mm                                  |
| obst.left, obst.right | clearance either side, mm, 0 unknown         |
| imu.*         | pitch, ok, head, rate, event, hit, stable, rough     |

| `action`               | When                                         |
|------------------------|----------------------------------------------|
| starting               | before the first state                       |
| following line         | on the line                                  |
| turning onto line      | crossing, creeping and turning back onto it  |
| line lost              | neither line sensor has seen the line lately |
| reading barcode        | taking a decoded command                     |
| stopped at junction    | waiting for a command at a cross             |
| turning left, turning right, u-turn | the junction's turn             |
| scanning obstacle      | profiling what is ahead                      |
| going round obstacle   | driving the detour                           |
| reversing to probe     | backing off to try a lane                    |
| backing out of lane    | undoing the legs of a blocked lane           |
| searching for line     | the search after a detour or a lost line     |
| backing off after hit  | after an IMU hit or a stalled wheel          |
| halted                 | stopped for good                             |

Terrain status on `car/terrain` every 2 s (`CAR_TERRAIN_PERIOD_MSEC`), for
`imu_terrain/terrain_report.md`:

| Key     | Meaning                                                   |
|---------|-----------------------------------------------------------|
| pitch   | degrees, nose up positive, frozen while `ok` is 0         |
| ok      | 1 while pitch can be believed                             |
| mag     | filtered acceleration magnitude, milli g                  |
| rough   | terrain roughness, milli g                                |
| terrain | STABLE or ROUGH                                           |
| event   | STILL, ACCEL, TURN, CLIMB, DESCEND or IMPACT              |
| on_hump | 1 while on a hump                                         |
| humps   | humps crossed this run                                    |
| last_mm | height of the most recent hump                            |
| peak_mm | height of the highest hump this run                       |

Heartbeat on `car/heartbeat` every second:
`{"uptime_ms":N,"connected":1,"sessions":N}`. `sessions` counts the
connections the broker has accepted, so anything above 1 means the car
lost the link and got it back.

Console mirror on `car/log`, radio builds only: every line the car or a
bench prints, as plain text, several whole lines to a message. Lines are
held while the car is offline and sent in order when it reconnects, up to
`LWIP_UTK_MQTT_LOG_BYTES` (8 KB). See `common/car_log.c`.

## Connection recovery

- **Broker lost** (stopped or unreachable): the MQTT session drops and the
  car tries again every `COMMS_RECONNECT_BACKOFF_MSEC` (5 s) until the
  broker answers.
- **Wi-Fi lost:** the radio service joins again 5 s after the link drops
  and keeps trying, each attempt timing out after 30 s. lwIP renews the
  address when the link returns, then MQTT reconnects as above.
- **While disconnected** nothing is queued for later: telemetry is live
  data, so the first message after reconnecting is current. The car keeps
  driving throughout, because comms has its own task.

The console prints `mqtt disconnected, session N` and `mqtt connected,
session N` at each change. To test it with the car or `./flash.sh --wifi
comms` running and `mosquitto_sub -h <broker ip> -t 'car/#' -v` open:

1. Stop the broker for 1 minute (`brew services stop mosquitto`, then
   `start`). Heartbeats stop, then resume with `sessions` 2.
2. Turn the access point or hotspot off for 30 s, then on. Heartbeats
   resume with `sessions` 3.

Time both from switching back on to the first heartbeat, into the table
below.

| Measurement                    | Value | Notes                        |
|--------------------------------|-------|------------------------------|
| Time to first MQTT connect     |       | from power on, seconds       |
| Reconnect time after drop      |       | pull the broker, time it     |
| Telemetry size per message     |       | 297 bytes typical on the host, 365 at most |
