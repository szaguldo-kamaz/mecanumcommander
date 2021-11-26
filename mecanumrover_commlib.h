/*
    NLAB-MecanumCommlib for Linux, a simple library to control VStone MecanumRover 2.1
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021
    version 0.40
    https://github.com/szaguldo-kamaz/
*/

//#define DEBUG 1

// to use the FTDI USB-UART on the robot controller
#define DEVFILE         "/dev/ttyUSB0"
#define BAUDRATE        B115200
// to use Raspberry Pi4's UART directly connected to the robot controller's UART (bypassing the FTDI chip - needs HW mod)
//#define DEVFILE         "/dev/ttyAMA1"
//#define BAUDRATE        B19200

#define BUFFER_SIZE     1024

// max speeds for the rover
#define LIMIT_SPEED_X   2100  // mm/sec
#define LIMIT_SPEED_Y   2100  // mm/sec
#define LIMIT_SPEED_ROT 8000  // mrad/sec

#define ROVER_CONTROLLER_ADDR_REAR  0x10
#define ROVER_CONTROLLER_ADDR_FRONT 0x1F

#define ROVER_REG_SYSTEMNAME        0x00
#define ROVER_REG_FIRMWAREREVISION  0x02
#define ROVER_REG_UPTIME            0x04
#define ROVER_REG_ENABLEMOTORS      0x10
#define ROVER_REG_OUTPUTOFFSET0     0x50
#define ROVER_REG_OUTPUTOFFSET1     0x52
#define ROVER_REG_MAXCURRENT0       0x58
#define ROVER_REG_MAXCURRENT1       0x5A
#define ROVER_REG_CURRENTLIMIT0     0x5C
#define ROVER_REG_CURRENTLIMIT1     0x5E
#define ROVER_REG_MEASUREDPOS0      0x60
#define ROVER_REG_MEASUREDPOS1      0x64
#define ROVER_REG_SPEED0            0x68
#define ROVER_REG_SPEED1            0x6A
#define ROVER_REG_MOTOROUTPUTCALC0  0x6C
#define ROVER_REG_MOTOROUTPUTCALC1  0x6E
#define ROVER_REG_BATTERYVOLTAGE    0x90
#define ROVER_REG_ENCODERVALUE0     0x98
#define ROVER_REG_ENCODERVALUE1     0x9C
#define ROVER_REG_MEASUREDCURRENT0  0xA0
#define ROVER_REG_MEASUREDCURRENT1  0xA2
#define ROVER_REG_SPEED_X           0xC0
#define ROVER_REG_SPEED_Y           0xC2
#define ROVER_REG_ROTATION          0xC4


struct roverstruct {
    unsigned  int sysname;
    unsigned  int firmrev;
    unsigned  int rs485_err_0x10;
    unsigned  int rs485_err_0x1F;
    unsigned char memmap_main[512];
    unsigned char memmap_front[512];
    unsigned char fullname[32];
    double uptime_sec;
    double battery_voltage;
    unsigned char main_motor_status;
    unsigned char front_motor_status;
    unsigned char motor_m1_status;
    unsigned char motor_m2_status;
    unsigned char motor_m3_status;
    unsigned char motor_m4_status;

    int front_speed0;
    int front_speed1;
    int main_speed0;
    int main_speed1;
    int front_measured_position0;
    int front_measured_position1;
    int main_measured_position0;
    int main_measured_position1;
    int front_encoder_value0;
    int front_encoder_value1;
    int main_encoder_value0;
    int main_encoder_value1;
    int front_outputoffset0;
    int front_outputoffset1;
    int main_outputoffset0;
    int main_outputoffset1;
    double front_motoroutput_calc0;
    double front_motoroutput_calc1;
    double main_motoroutput_calc0;
    double main_motoroutput_calc1;
    double front_measured_current_value0;
    double front_measured_current_value1;
    double main_measured_current_value0;
    double main_measured_current_value1;
    double front_max_current0;
    double front_max_current1;
    double main_max_current0;
    double main_max_current1;
    double front_current_limit0;
    double front_current_limit1;
    double main_current_limit0;
    double main_current_limit1;

};

int conv_int16_to_int32(int int16);
int check_and_remove_rs485_error(unsigned char *message);
int check_invalidchars(unsigned char *message);
int check_and_remove_readey(unsigned char *message); // readey (sic!)

// serial port - transmit
int send_command_raw(unsigned char *message, unsigned char messagelen, unsigned char *reply);
// hexstring->endianness->num
int read_register_from_memmap(unsigned char *memmap, unsigned char addr, unsigned char reglen);
// read only 1 register, and update it in the memmap
int rover_read_register(unsigned char id, unsigned char addr, unsigned char length, unsigned char *memmap, struct roverstruct *rover);

int rover_write_register_uint8(unsigned char id, unsigned char addr, unsigned char data, unsigned char *reply);
int rover_write_register_uint16(unsigned char id, unsigned char addr, unsigned int data, unsigned char *reply);
int rover_write_register_int16(unsigned char id, unsigned char addr, int data, unsigned char *reply);
int rover_write_register_uint32(unsigned char id, unsigned char addr, unsigned int data, unsigned char *reply);
int rover_read_full_memmap(unsigned char *memmap, unsigned int controller_id, struct roverstruct *rover);

// get values from previously read memmap
int    rover_get_sysname(unsigned char *memmap);
int    rover_get_firmrev(unsigned char *memmap);
double rover_get_uptime(unsigned char *memmap);
unsigned char rover_get_motor_status(unsigned char *memmap);
int    rover_get_outputoffset0(unsigned char *memmap);
int    rover_get_outputoffset1(unsigned char *memmap);
double rover_get_max_current0(unsigned char *memmap);
double rover_get_max_current1(unsigned char *memmap);
double rover_get_current_limit0(unsigned char *memmap);
double rover_get_current_limit1(unsigned char *memmap);
int    rover_get_measured_position0(unsigned char *memmap);
int    rover_get_measured_position1(unsigned char *memmap);
int    rover_get_speed0(unsigned char *memmap);
int    rover_get_speed1(unsigned char *memmap);
double rover_get_motoroutput_calc0(unsigned char *memmap);
double rover_get_motoroutput_calc1(unsigned char *memmap);
double rover_get_battery_voltage(unsigned char *memmap);
int    rover_get_encoder_value0(unsigned char *memmap);
int    rover_get_encoder_value1(unsigned char *memmap);
double rover_get_measured_current_value0(unsigned char *memmap);
double rover_get_measured_current_value1(unsigned char *memmap);
int    rover_get_X_speed(unsigned char *memmap);
int    rover_get_Y_speed(unsigned char *memmap);
int    rover_get_rotation_speed(unsigned char *memmap);

// write commands
int rover_enable_motors_main(unsigned char *reply);
int rover_enable_motors_front(unsigned char *reply);
int rover_disable_motors_main(unsigned char *reply);
int rover_disable_motors_front(unsigned char *reply);

int rover_set_X_speed(int speed, unsigned char *reply);
int rover_set_Y_speed(int speed, unsigned char *reply);
int rover_set_rotation_speed(int speed, unsigned char *reply);
int rover_set_XYrotation_speed_to_zero(unsigned char *reply);

int rover_set_encoder_value0(unsigned int value, unsigned char *reply);
int rover_set_encoder_value1(unsigned int value, unsigned char *reply);
int rover_set_measured_position0(unsigned int value, unsigned char *reply);
int rover_set_measured_position1(unsigned int value, unsigned char *reply);

// kkk commands - more robust comm
// needs custom firmware!
int rover_kset_STOP(unsigned char *reply);
int rover_kset_XYrotation_speed(int xspeed, int yspeed, int rotspeed, unsigned char *reply);
