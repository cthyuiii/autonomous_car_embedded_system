# line_barcode

Owns the three IR sensors, calibration, line position, junction detection,
barcode decoding and the navigation command it produces. API is
`include/line_barcode.h`. Knobs are `LINE_*` and `TRACK_*` in
`common/car_config.h`. Decide digital or analog first, see the header.

Done means `test_line_barcode.c` passes on the host, `bench_line_barcode`
shows a clean mask sweep by hand, and all four barcodes decode correctly
ten times each at the speed the car will actually run.

| Calibration                 | Measured | How                            |
|-----------------------------|----------|--------------------------------|
| Sensor ride height, mm      |          | TCRT5000 peaks at 2.5 mm       |
| Dark and light reading      |          | per sensor, both surfaces      |
| Narrow and wide bar samples |          | count at run speed             |
| Max speed that still decodes|          | raise until a symbol misreads  |
