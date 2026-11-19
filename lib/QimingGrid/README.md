# QimingGrid

Production nonblocking client for Qiming public protocol v6 and a 25-cell grid
mission. This library has no Arduino/HAL dependency; the actual firmware UART,
IMU and wheel bridge lives in `src/radar_navigation.inc`.

`link_protocol.h` and `LinkProtocol.cpp` are copied byte for byte from the
supplied Qiming F407 example's `radar_example/protocol/link_protocol.h/.c`.
The C source is compiled as C++ to keep consistent linkage. The other files
are the project's client, geometry and mission implementation.

Run `python3 test/qiming_grid_test.py` from the repository root for real encoded
frame simulation, geometry, both-zone missions and the actual drive bridge.
Setup, coordinates, protocol assumptions and practical limits are documented in
`docs/RADAR_GRID_NAVIGATION.md`.
