# line_barcode

Owner: Buddy 3, barcode decoding and IR line following.

Owns the three IR sensors, line position, junction detection, Code 39
barcode decoding and the navigation command it produces. API is
`include/line.h`. Knobs are `LINE_*`, `TRACK_*` and `BARCODE_*` in
`common/car_config.h`.

Implemented for three MH-Sensor-Series digital modules. Left (Grove 5)
and right (Grove 2) sit at the front and straddle the line, about 25 to
30 mm apart, so both see floor on a straight. The third (Grove 6) sits off
to the left and only reads barcodes. Position is -2, 0 or 2; neither sensor
dark counts as centred for `LINE_CENTRED_HOLD_MSEC` after the last
sighting, then as lost. The decoder
times every bar and space the barcode sensor sees, sampled every
`BARCODE_SAMPLE_USEC` by a timer interrupt rather than the 10 ms line task,
classifies the three widest of each nine as wide, checks both asterisks,
and tries the sequence reversed, so a symbol reads from either end at
any speed the car can reach. `LINE_SENSOR_DARK_LEVEL`
is the one thing to confirm first: if the mask reads inverted, flip it.

`line_get_raw_levels()` is the first thing to read: it is the electrical
level on each pin before the dark level mapping, so it moves whichever way
round a module's output is wired, and the bench counts every flip per
sensor. A counter that never moves means nothing is reaching that pin.

`line_get_health()` is the next: a sensor only counts as
working once it has been seen both dark and light, and the bench prints
its letter in capitals when it has. Lower case letters with a steady
`mask 1x1` is sensors that have never changed, which is wiring, not
a junction. The barcode letter only capitalises once it crosses a bar.

Done means `test_line_barcode.c` passes on the host, all three health
letters capitalise, `bench_line_barcode` shows a clean mask sweep by hand, and all four barcodes decode correctly
ten times each at the speed the car will actually run.

| Calibration                 | Measured | How                            |
|-----------------------------|----------|--------------------------------|
| Sensor ride height, mm      |          | widest contrast on the bench   |
| LINE_SENSOR_DARK_LEVEL      |          | mask over tape vs floor        |
| Trimpot per module          |          | LED steady over floor, lit on the line; no `line sensors .. read dark at the start` at boot |
| Max speed that still decodes|          | raise until a symbol misreads  |
| BARCODE_CHAR_* mapping      | A B C D  | from the course write-up       |
