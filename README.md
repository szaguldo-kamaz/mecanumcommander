# mecanumcommander
Simplified interface for controlling the Vstone MecanumRover robot (メカナムローバー)

**mecanumrover_commander** is an ncurses-based UI to control the rover. Use the arrow or WASD keys to control the speed of the robot, rotation can be controller with the Home/PgUp keys.
Holding "shift" with the mentioned keys will give a boost to the speed increase/decrease. The "space" key (and some others also) sends a stop command to the rover.
Also listens on TCP/3475 and accepts text commands like:

* `SPX01000` -> set X speed to 1000mm/s (+ forward, - backward)
* `SPY-0570` -> set Y speed to -570mm/s (+ left, - right)
* `ROT02000` -> set rotation speed to 2000mrad/s (+ ccw, - cw)
* `STOPZERO` -> set X/Y/rotation speed to zero in one step (stop the robot)

Remote commands have to be repeated at least every 500ms, otherwise the robot will stop intentionally (failsafe).

**mecanumrover_commlib** is a library to provide the low-level functions for communicating with the MecanumRover through its serial interface.
The robot uses a "memory map" to represent the robot's state.
This library gives an abstraction layer for accessing the registers of this memory map, while also providing data conversion and simple error checking methods.

**mecanumrover_monitor** can be used to periodically read and display some useful parameters from the robot's controller.

**memmapupdate_via_wifi.sh** A shell script to download the memory map via http (wget is required) if the rover is connected to the wifi network.
The IP addr of the rover is read from the `ROVERIP` environment variable, e.g.:
`env ROVERIP=192.168.0.123 /bin/bash memmapupdate_via_wifi.sh`
