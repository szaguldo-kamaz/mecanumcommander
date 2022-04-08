/*
    NLAB-MecanumCommlib for Linux, a simple library to control VStone MecanumRover 2.1
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021
    version 0.40
    https://github.com/szaguldo-kamaz/
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "mecanumrover_commlib.h"

int main() {

    int ret, i=0;
    double rover_uptime_sec, rover_battery_voltage;
    int rover_sysname, rover_firmrev, rover_main_motor_status, rover_front_motor_status;

    unsigned char answer[BUFFER_SIZE];
    struct roverstruct rover;

    ret = rover_read_full_memmap(rover.memmap_main, ROVER_CONTROLLER_ADDR_REAR, &rover);

    if (rover_identify(&rover, rover.memmap_main) == 1) {
        printf("Unknown rover!\n");
        exit(1);
    }

    rover_uptime_sec = rover_get_uptime(rover.memmap_main);
    rover_battery_voltage = rover_get_battery_voltage(rover.memmap_main);
    rover_main_motor_status = rover_get_motor_status(rover.memmap_main);

    printf("Rover ID: 0x%x Firmware rev: 0x%x Uptime: %lf sec Battery: %lf V \n",
            rover.sysname, rover.firmrev, rover_uptime_sec, rover_battery_voltage);

    if (rover.has_second_controller == 1) {
        ret = rover_read_full_memmap(rover.memmap_front, ROVER_CONTROLLER_ADDR_FRONT, &rover);
        rover_front_motor_status = rover_get_motor_status(rover.memmap_front);
    }

    if (rover.motor_count == 2) {
        printf("Motor status: %d\n", rover_main_motor_status);
        printf("MaxCurrent0,1 (Amps): % 7.2lf,% 7.2lf\n",
         rover_get_max_current0(rover.memmap_main), rover_get_max_current1(rover.memmap_main));
        printf("CurrentLimit0,1 (Amps): % 7.2lf,% 7.2lf\n",
         rover_get_current_limit0(rover.memmap_main), rover_get_current_limit1(rover.memmap_main));
    } else {
        if (rover.motor_count == 4) {
            printf("Motor status (main/front): %d/%d\n", rover_main_motor_status, rover_front_motor_status);
            printf("MaxCurrent0,1,2,3 (Amps): % 7.2lf,% 7.2lf,% 7.2lf,% 7.2lf\n",
             rover_get_max_current0(rover.memmap_main),  rover_get_max_current1(rover.memmap_main),
             rover_get_max_current0(rover.memmap_front), rover_get_max_current1(rover.memmap_front));
            printf("CurrentLimit0,1,2,3 (Amps): % 7.2lf,% 7.2lf,% 7.2lf,% 7.2lf\n",
             rover_get_current_limit0(rover.memmap_main),  rover_get_current_limit1(rover.memmap_main),
             rover_get_current_limit0(rover.memmap_front), rover_get_current_limit1(rover.memmap_front));
        }
    }

    while (1) {

        if (i++ % 15 == 0) {
            switch (rover.motor_count) {
                case 2:
                    printf("-------------------------------------------------------------------------------------------------------------------------------\n");
                    printf(" Uptime  Batt(V) Mot| Pos0, Pos1 | Enc0, Enc1 |  Spd0,  Spd1 |  OutputOffset0,1  |     MotOutCalc0,1     |  MeasuredCurr0,1    \n");
                    printf("-------------------------------------------------------------------------------------------------------------------------------\n");
                    break;

                case 4:
                    printf("------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
                    printf(" Uptime  Batt(V) Mot| Pos0, Pos1, Pos2, Pos3| Enc0, Enc1, Enc2, Enc3|  Spd0,  Spd1,  Spd2,  Spd3|    OutputOffset0,1,2,3    |        MotOutCalc0,1,2,3      |  MeasuredCurr0,1,2,3  \n");
                    printf("------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
                    break;
            }
        }

        ret = rover_read_full_memmap(rover.memmap_main, ROVER_CONTROLLER_ADDR_REAR, &rover);

        rover_uptime_sec = rover_get_uptime(rover.memmap_main);
        rover_battery_voltage = rover_get_battery_voltage(rover.memmap_main);
        rover_main_motor_status = rover_get_motor_status(rover.memmap_main);

        if (rover.has_second_controller == 1) {
            ret = rover_read_full_memmap(rover.memmap_front, ROVER_CONTROLLER_ADDR_FRONT, &rover);
            rover_front_motor_status = rover_get_motor_status(rover.memmap_front);
        }

        switch (rover.motor_count) {
            case 2:
                printf("% 6.2lf  %2.2lf  %d|% 5d,% 5d|% 5d,% 5d|% 6d,% 6d|% 6d,% 6d|"
                       "% 7.2lf,% 7.2lf|% 4.2lf,% 4.2lf\n",
                    rover_uptime_sec, rover_battery_voltage, rover_main_motor_status,
                    rover_get_measured_position0(rover.memmap_main), rover_get_measured_position1(rover.memmap_main),
                    rover_get_encoder_value0(rover.memmap_main), rover_get_encoder_value1(rover.memmap_main),
                    rover_get_speed0(rover.memmap_main), rover_get_speed1(rover.memmap_main),
                    rover_get_outputoffset0(rover.memmap_main), rover_get_outputoffset1(rover.memmap_main),
                    rover_get_motoroutput_calc0(rover.memmap_main), rover_get_motoroutput_calc1(rover.memmap_main),
                    rover_get_measured_current_value0(rover.memmap_main), rover_get_measured_current_value1(rover.memmap_main)
                );
                break;

            case 4:
                printf("% 6.2lf  %2.2lf  %d/%d|% 5d,% 5d,% 5d,% 5d|% 5d,% 5d,% 5d,% 5d|% 6d,% 6d,% 6d,% 6d|% 6d,% 6d,% 6d,% 6d|"
                       "% 7.2lf,% 7.2lf,% 7.2lf,% 7.2lf|% 4.2lf,% 4.2lf,% 4.2lf,% 4.2lf\n",
                    rover_uptime_sec, rover_battery_voltage, rover_main_motor_status, rover_front_motor_status,
                    rover_get_measured_position0(rover.memmap_main),  rover_get_measured_position1(rover.memmap_main),
                    rover_get_measured_position0(rover.memmap_front), rover_get_measured_position1(rover.memmap_front),
                    rover_get_encoder_value0(rover.memmap_main),  rover_get_encoder_value1(rover.memmap_main),
                    rover_get_encoder_value0(rover.memmap_front), rover_get_encoder_value1(rover.memmap_front),
                    rover_get_speed0(rover.memmap_main),  rover_get_speed1(rover.memmap_main),
                    rover_get_speed0(rover.memmap_front), rover_get_speed1(rover.memmap_front),
                    rover_get_outputoffset0(rover.memmap_main),  rover_get_outputoffset1(rover.memmap_main),
                    rover_get_outputoffset0(rover.memmap_front), rover_get_outputoffset1(rover.memmap_front),
                    rover_get_motoroutput_calc0(rover.memmap_main),  rover_get_motoroutput_calc1(rover.memmap_main),
                    rover_get_motoroutput_calc0(rover.memmap_front), rover_get_motoroutput_calc1(rover.memmap_front),
                    rover_get_measured_current_value0(rover.memmap_main),  rover_get_measured_current_value1(rover.memmap_main),
                    rover_get_measured_current_value0(rover.memmap_front), rover_get_measured_current_value1(rover.memmap_front)
                );
                break;
        }

    }

    return 0;

}
