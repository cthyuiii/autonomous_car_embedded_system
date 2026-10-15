# line_barcode

Owner: Buddy 3, barcode decoding and IR line following.

Owns the three IR sensors, line position, junction detection, Code 39
decoding and the command it produces. The API is in `include/line.h`, the
values are `LINE_*`, `TRACK_*` and `BARCODE_*` in `common/car_config.h`.

Three MH-Sensor-Series digital modules. Left (Grove 5) and right (Grove 2)
straddle the line with their eyes 35 mm apart, so both see floor on a
straight. The third (Grove 6) sits to the left and only reads barcodes.
Position is -2, 0 or 2. With neither sensor dark the line counts as
centred for `LINE_CENTRED_HOLD_MSEC`, then as lost. A junction is each
line sensor dark within `LINE_JUNCTION_WINDOW_MSEC`.

The decoder times every bar and space, sampled every `BARCODE_SAMPLE_USEC`
by a timer interrupt, takes the three widest of each nine as wide, checks
both asterisks and also tries the sequence reversed, so a barcode reads
from either end at any speed the car reaches.

Read `line_get_raw_levels()` first: the level on each pin before the dark
level mapping. The bench counts every flip per sensor, and a counter that
never moves means nothing reaches that pin. Then `line_get_health()`: a
sensor counts as working once it has been seen both dark and light, and
the bench prints its letter in capitals. Lower case letters with a steady
`mask 1x1` mean wiring, not a junction.

Tests: `test_line_barcode.c` on the host, `./flash.sh line_barcode` on the
car.

| Calibration            | Value      | How                                    |
|------------------------|------------|----------------------------------------|
| LINE_SENSOR_DARK_LEVEL | 1          | mask over tape against floor. Flip it if the mask reads inverted |
| Trimpot per module     | set by hand | LED steady over floor and lit on the line, with no `read dark at the start` at boot |
| BARCODE_CHAR_*         | A B C D    | A left, B right, C straight, D U-turn  |
| Barcodes decoded       | all four   | on the bench                           |
