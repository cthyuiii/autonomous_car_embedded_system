# comms

Owner: Buddy 1, WiFi communication, command and telemetry.

Owns WiFi association, the MQTT session, telemetry and heartbeat publishing,
command subscription and reconnection. API is `include/comms.h`. Knobs are
the `COMMS_*` constants in `common/car_config.h`.

Done means `test_comms.c` passes on the host, and `bench_comms` stays
connected for ten minutes, recovers within 30 seconds after the broker is
restarted, and echoes every command sent from a laptop MQTT client.

Document the telemetry JSON keys here once `comms_publish_telemetry` exists.

| Measurement                    | Value | Notes                        |
|--------------------------------|-------|------------------------------|
| Time to first MQTT connect     |       | from power on, seconds       |
| Reconnect time after drop      |       | pull the broker, time it     |
| Telemetry size per message     |       | bytes, keep it small         |
