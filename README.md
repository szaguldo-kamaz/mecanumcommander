# mecanumcommander
Simplified interface for controlling the [Vstone MecanumRover v2.1 robot (メカナムローバー)](https://www.vstone.co.jp/products/wheelrobot/ver2.1.html#mecanumrover2.1) and the [Vstone MegaRover v3 robot (メガローバー)](https://www.vstone.co.jp/products/wheelrobot/ver.3.0_normal.html)

**mecanumrover_commander** is an ncurses-based UI to control the MecanumRover v2.1 / MegaRover v3, but can be easily extended to support other Vstone robots.
Use the arrow or WASD keys to control the speed of the robot, rotation can be controlled with the Home/PgUp or Q/E keys.
Holding "Shift" with the mentioned keys will give a boost to the speed increase/decrease. The "space" key (and some others also, like X, Del) sends a stop command to the rover.
Can also listen on UDP/TCP port 3475 and accepts simple text commands over the network, like:

* `SPX01000` -> set X speed to 1000mm/s (+ forward, - backward)
* `SPY-0570` -> set Y speed to -570mm/s (+ left, - right)
* `ROT02000` -> set rotation speed to 2000mrad/s (+ ccw, - cw)
* `STOPZERO` -> set X/Y/rotation speed to zero in one step (stop the robot)

Remote commands have to be repeated at least every 500ms, otherwise the robot will stop intentionally (to reduce the risk of causing damage).
For TCP you can easily use netcat/telnet for testing, for UDP you will also need to add a packet counter and an extra CRC checksum (see client example).

![mecacom040_screenshot](https://user-images.githubusercontent.com/86873213/133548313-0c7746d7-e2b6-4c1c-8e45-a02d7f5e305a.png)

**mecanumrover_commlib** is a library to provide the low-level functions for communicating with the MecanumRover / MegaRover through its serial interface.
The robot uses a "memory map" to represent the robot's state.
This library gives an abstraction layer for accessing the registers of this memory map, while also providing data conversion and simple error checking methods.

**mecanumrover_monitor** can be used to periodically read and display some useful parameters from the robot's controller.

**mecanumrover_memmap_dump_to_file** reads the main memmap of the robot's controller and saves it to a file (memmap_dump.dat).

**memmapupdate_via_wifi.sh** A shell script to download the memory map via http (wget is required) if the rover is connected to the wifi network.
The IP addr of the rover is read from the `ROVERIP` environment variable, e.g.: \
`env ROVERIP=192.168.0.123 /bin/bash memmapupdate_via_wifi.sh`

**client/** A Python client example, both for UDP/TCP.
