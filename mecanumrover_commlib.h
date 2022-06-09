/*
    NLAB-MecanumCommlib for Linux, a simple library to control VStone MecanumRover 2.1 / VStone MegaRover 3
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021
    version 0.50
    https://github.com/szaguldo-kamaz/
*/

//#define DEBUG 1

#ifndef __MECACOMLIB_H__

#define __MECACOMLIB_H__

// to use the FTDI USB-UART on the robot controller
#define DEVFILE          "/dev/ttyUSB0"
#define BAUDRATE         B115200
// to use Raspberry Pi4's UART directly connected to the robot controller's UART (bypassing the FTDI chip - needs HW mod)
//#define DEVFILE          "/dev/ttyAMA1"
//#define BAUDRATE         B19200

#define BUFFER_SIZE      1024

// max speeds for the rover
#define LIMIT_SPEED_X    2100  // mm/sec
#define LIMIT_SPEED_Y    2100  // mm/sec
#define LIMIT_SPEED_ROT  8000  // mrad/sec

// max time in usec to wait for reply when sending a command to the rover's controller
//#define REPLYWAIT_TIMEOUT_USEC 100000
#define REPLYWAIT_TIMEOUT_USEC 50000

#define SYSNAME_MECANUMROVER21      0x21
#define SYSNAME_MEGAROVER3          0x30

#define CONTROLLER_ADDR_DEFAULT     0x10
#define CONTROLLER_ADDR_MAIN        0x10
#define CONTROLLER_ADDR_SECOND      0x1F

#define ROVER_REG_SYSTEMNAME        0x00
#define ROVER_REG_FIRMWAREREVISION  0x02
#define ROVER_REG_UPTIME            0x04
#define ROVER_REG_ENABLEMOTORS      0x10

#define MECANUMROVER21_REG_OUTPUTOFFSET0     0x50
#define MECANUMROVER21_REG_OUTPUTOFFSET1     0x52
#define MECANUMROVER21_REG_MAXCURRENT0       0x58
#define MECANUMROVER21_REG_MAXCURRENT1       0x5A
#define MECANUMROVER21_REG_CURRENTLIMIT0     0x5C
#define MECANUMROVER21_REG_CURRENTLIMIT1     0x5E
#define MECANUMROVER21_REG_MEASUREDPOS0      0x60
#define MECANUMROVER21_REG_MEASUREDPOS1      0x64
#define MECANUMROVER21_REG_SPEED0            0x68
#define MECANUMROVER21_REG_SPEED1            0x6A
#define MECANUMROVER21_REG_MOTOROUTPUTCALC0  0x6C
#define MECANUMROVER21_REG_MOTOROUTPUTCALC1  0x6E
#define MECANUMROVER21_REG_BATTERYVOLTAGE    0x90
#define MECANUMROVER21_REG_ENCODERVALUE0     0x98
#define MECANUMROVER21_REG_ENCODERVALUE1     0x9C
#define MECANUMROVER21_REG_MEASUREDCURRENT0  0xA0
#define MECANUMROVER21_REG_MEASUREDCURRENT1  0xA2
#define MECANUMROVER21_REG_SPEED_X           0xC0
#define MECANUMROVER21_REG_SPEED_Y           0xC2
#define MECANUMROVER21_REG_ROTATION          0xC4

#define MEGAROVER3_REG_ENCODERVALUE0         0x50
#define MEGAROVER3_REG_ENCODERVALUE1         0x54
#define MEGAROVER3_REG_MOTORSPEED0           0x60
#define MEGAROVER3_REG_MOTORSPEED1           0x64
#define MEGAROVER3_REG_MEASUREDCURRENT0      0x70
#define MEGAROVER3_REG_MEASUREDCURRENT1      0x72
#define MEGAROVER3_REG_BATTERYVOLTAGE        0x82
#define MEGAROVER3_REG_SPEED_X               0x90
#define MEGAROVER3_REG_SPEED_Y               0x92 // unused
#define MEGAROVER3_REG_ROTATION              0x94


struct rover_config {
    unsigned char has_second_controller;
    unsigned char has_Y_speed;
    unsigned char motor_count;
    unsigned char enablemotors_on;
    unsigned char enablemotors_off;
};

struct rover_regs {
    unsigned char controller_addr_main;
    unsigned char controller_addr_second;

    unsigned char batteryvoltage;
    unsigned char speed_x;
    unsigned char speed_y;
    unsigned char rotation;
    unsigned char enablemotors;

    unsigned char systemname;
    unsigned char firmwarerevision;
    unsigned char uptime;

    unsigned char outputoffset0;
    unsigned char outputoffset1;
    unsigned char motoroutputcalc0;
    unsigned char motoroutputcalc1;

    unsigned char maxcurrent0;
    unsigned char maxcurrent1;
    unsigned char currentlimit0;
    unsigned char currentlimit1;
    unsigned char measuredcurrent0;
    unsigned char measuredcurrent1;

    unsigned char measuredpos0;
    unsigned char measuredpos1;
    unsigned char speed0;
    unsigned char speed1;
    unsigned char encodervalue0;
    unsigned char encodervalue1;

    unsigned char motorspeed0;
    unsigned char motorspeed1;
};

struct roverstruct {
    struct rover_config *config;
    struct rover_regs *regs;
    unsigned  int sysname;
    unsigned  int firmrev;
    unsigned  int rs485_err_0x10;
    unsigned  int rs485_err_0x1F;
    unsigned char memmap_main[512];
    unsigned char memmap_second[512];
    unsigned char fullname[32];
};

int conv_int16_to_int32(int int16);
int check_and_remove_rs485_error(unsigned char *message);
int check_invalidchars(unsigned char *message);
int check_and_remove_readey(unsigned char *message); // readey (sic!)
int check_serial_dev();

// serial port - transmit
int send_command_raw(unsigned char *message, unsigned char messagelen, unsigned char *reply);
// hexstring->endianness->num
int read_register_from_memmap(unsigned char *memmap, unsigned char register_addr, unsigned char register_length);
// read only 1 register, and update it in the memmap
int rover_read_register(unsigned char controller_addr, unsigned char register_addr, unsigned char length, unsigned char *memmap, struct roverstruct *rover);

int rover_write_register_uint8(unsigned char controller_addr, unsigned char register_addr, unsigned char data, unsigned char *reply);
int rover_write_register_uint16(unsigned char controller_addr, unsigned char register_addr, unsigned int data, unsigned char *reply);
int rover_write_register_int16(unsigned char controller_addr, unsigned char register_addr, int data, unsigned char *reply);
int rover_write_register_uint32(unsigned char controller_addr, unsigned char register_addr, unsigned int data, unsigned char *reply);
int rover_read_full_memmap(unsigned char *memmap, unsigned int controller_addr, struct roverstruct *rover);

unsigned int rover_get_controller_addr(struct roverstruct *rover, unsigned int controller_id);

unsigned char rover_identify(struct roverstruct *rover);
unsigned char rover_identify_from_main_memmap(struct roverstruct *rover);

// get values from previously read memmap

int    rover_get_sysname(struct roverstruct *rover);
int    rover_get_firmrev(struct roverstruct *rover);
double rover_get_uptime(struct roverstruct *rover);
double rover_get_battery_voltage(struct roverstruct *rover);
int    rover_get_X_speed(struct roverstruct *rover);
int    rover_get_Y_speed(struct roverstruct *rover);
int    rover_get_rotation_speed(struct roverstruct *rover);

unsigned char rover_get_motor_status(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_outputoffset0(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_outputoffset1(struct roverstruct *rover, unsigned char *memmap);
double rover_get_max_current0(struct roverstruct *rover, unsigned char *memmap);
double rover_get_max_current1(struct roverstruct *rover, unsigned char *memmap);
double rover_get_current_limit0(struct roverstruct *rover, unsigned char *memmap);
double rover_get_current_limit1(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_measured_position0(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_measured_position1(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_speed0(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_speed1(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_motorspeed0(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_motorspeed1(struct roverstruct *rover, unsigned char *memmap);
double rover_get_motoroutput_calc0(struct roverstruct *rover, unsigned char *memmap);
double rover_get_motoroutput_calc1(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_encoder_value0(struct roverstruct *rover, unsigned char *memmap);
int    rover_get_encoder_value1(struct roverstruct *rover, unsigned char *memmap);
double rover_get_measured_current_value0(struct roverstruct *rover, unsigned char *memmap);
double rover_get_measured_current_value1(struct roverstruct *rover, unsigned char *memmap);

// write commands
int rover_enable_motors( struct roverstruct *rover, unsigned char controller_addr, unsigned char *reply);
int rover_disable_motors(struct roverstruct *rover, unsigned char controller_addr, unsigned char *reply);

int rover_set_X_speed(struct roverstruct *rover, int speed_x, unsigned char *reply);
int rover_set_Y_speed(struct roverstruct *rover, int speed_y, unsigned char *reply);
int rover_set_rotation_speed(struct roverstruct *rover, int speed_rot, unsigned char *reply);
int rover_set_XYrotation_speed_to_zero(struct roverstruct *rover, unsigned char *reply);

// kkk commands - more robust comm
// needs custom firmware!
int rover_kset_STOP(unsigned char *reply);
int rover_kset_XYrotation_speed(int xspeed, int yspeed, int rotspeed, unsigned char *reply);

#endif
