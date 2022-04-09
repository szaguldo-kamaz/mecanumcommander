/*
    NLAB-MecanumCommlib for Linux, a simple library to control VStone MecanumRover 2.1
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021
    version 0.40
    https://github.com/szaguldo-kamaz/
*/

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include "mecanumrover_commlib.h"


const struct rover_regs rover_regs_unknown = {
    0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x04,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const struct rover_regs rover_regs_mecanumrover21 = {
    0x10, 0x1F,
    0x90, 0xC0, 0xC2, 0xC4, 0x10,
    0x00, 0x02, 0x04,
    0x50, 0x52, 0x6C, 0x6E,
    0x58, 0x5A, 0x5C, 0x5E, 0xA0, 0xA2,
    0x60, 0x64, 0x68, 0x6A, 0x98, 0x9C
};

const struct rover_regs rover_regs_megarover3 = {
    0x10, 0x10,
    0x82, 0x90, 0x92, 0x94, 0x10,
    0x00, 0x02, 0x04,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x50, 0x54
};

int conv_int16_to_int32(int int16) {
    if (int16 > 32767) { int16 = int16 - 65536; }
    return int16;
}


int check_and_remove_rs485_error(unsigned char *message) {
// 485err_T: 1F\r\n
// 485err_T: 10\r\n
    int i, j, messagelen;

    messagelen = strlen(message);
    for (i = 0; ((message[i] != 'e') && (i < messagelen)); i++) {}
    if (i == messagelen) { // not found - OK
        return 0;
    }
    if (i > (messagelen - 11)) { // unknown or partial error msg
        return -2;
    }
    if (strncmp(&message[i-3], "485err_T: 10\r\n", 14) == 0) {
        for (j = i - 3; (message[j+14] != 0); j++) { message[j] = message[j+14]; }
        message[j] = 0;
        return 0x10;
    }
    if (strncmp(&message[i-3], "485err_T: 1F\r\n", 14) == 0) {
        for (j = i - 3; (message[j+14] != 0); j++) { message[j] = message[j+14]; }
        message[j] = 0;
        return 0x1F;
    }
    return -1; // unknown or partial error msg
}


int check_and_remove_readey(unsigned char *message) {
// readey\r\n
    int i, j, messagelen;

    messagelen = strlen(message);
    for (i = 0; ((message[i] != 'y') && (i < messagelen)); i++) {}
    if (i == messagelen) { // not found - OK
        return 0;
    }
    if (i > (messagelen - 2)) { // it can't be "readey" with this length, maybe another msg?
        return -2;
    }
    if (strncmp(&message[i-5], "readey\r\n", 8) == 0) {
        for (j = i - 5; (message[j+8] != 0); j++) { message[j] = message[j+8]; }
        message[j] = 0;
        return 1;
    }
    return -1; // unknown msg
}


int check_invalidchars(unsigned char *message) {
    int i, j;

    for (i = 0; (i < strlen(message)); i++) {
        if ( !( ((message[i] >= 48) && (message[i] <= 57)) || ((message[i] >= 65) && (message[i] <= 70)) || (message[i] == 10) || (message[i] == 13) ) ) {
            return -1;
        }
    }
    return 0;
}


int send_command_raw(unsigned char *message, unsigned char messagelen, unsigned char *reply) {

    int serial, ret;
    unsigned char incoming, datareceived;
    struct termios tio;
    fd_set serfdset;
    struct timeval tv;


    serial = open(DEVFILE, O_RDWR | O_NOCTTY);
    if (serial == -1) {
        perror("open(" DEVFILE "): ");
        return -1;
    }

    bzero(&tio, sizeof(tio));

    tio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    tio.c_cc[VTIME] = 0;     /* inter-character timer unused */
    tio.c_cc[VMIN]  = 1;     /* blocking read until 1 character arrives */

    ret = tcflush(serial, TCIFLUSH);
    if (ret != 0) {
        perror("tcflush(): ");
    }
    ret = tcsetattr(serial, TCSANOW, &tio);
    if (ret != 0) {
        perror("tcsetattr(): ");
    }

    ret = -1;
    FD_ZERO(&serfdset);
    FD_SET(serial, &serfdset);
    tv.tv_sec  = 0;
    tv.tv_usec = 100000;
//    tv.tv_usec = 10000;

    ret = write(serial, message, ++messagelen);
    if (ret != messagelen) {
        printf("write() returned fewer bytes than expected!\n");
        if (ret == -1) {
            perror("write(serial): ");
        }
    }

    datareceived = 0;

    reply[0] = 0;
    while(1) {
        ret = select(serial + 1, &serfdset, NULL, NULL, &tv);
        if (ret == -1) {
           perror("select(): ");
           break;
        }
        else if (ret) {
            ret = read(serial, &incoming, 1);
#ifdef DEBUG
            printf("Reply: 0x%x\n", incoming);
#endif
            // ha esetleg van meg valami egyeb a kovetkezo sorban, amugy ne varjunk ha nincs a rendes valasz utan semmi
            if (incoming == '\n') {
                tv.tv_usec = 0;
            } else {
                tv.tv_usec = 100000;
            }
            reply[datareceived++] = incoming;
            if (datareceived == BUFFER_SIZE) {
                printf("Buffer full (%d bytes)!\n", BUFFER_SIZE);
                close(serial);
                return datareceived;
            }
        } else {
#ifdef DEBUG
            if (datareceived == 0) {
                printf("No reply within 100000 usec.\n");
            }
#endif
            break;
        }
    }

#ifdef DEBUG
    if (datareceived > 0) {
        printf("\n");
    }
#endif

    close(serial);

    reply[datareceived] = 0;

    return datareceived;
}


// hexstring->endianness->num
int read_register_from_memmap(unsigned char *memmap, unsigned char addr, unsigned char reglen) {

    unsigned char reply[16];
    int regvalue, ret;

    if ( (reglen != 1) && (reglen != 2) && (reglen != 4) ) {
        printf("read_register_from_memmap(addr:0x%x): Invalid length value (%d)! Valid reglen values are: 1, 2, 4.\n", addr, reglen);
        return -1; // -1 is ok here, because this is the programmer's mistake...
    }

    if (reglen == 1) {
        reply[0] = memmap[addr*2+0];
        reply[1] = memmap[addr*2+1];
        reply[2] = 0;
    } else {
        if (reglen == 2) {
            reply[0] = memmap[addr*2+2];
            reply[1] = memmap[addr*2+3];
            reply[2] = memmap[addr*2+0];
            reply[3] = memmap[addr*2+1];
            reply[4] = 0;
        } else {
            if (reglen == 4) {
                reply[0] = memmap[addr*2+6];
                reply[1] = memmap[addr*2+7];
                reply[2] = memmap[addr*2+4];
                reply[3] = memmap[addr*2+5];
                reply[4] = memmap[addr*2+2];
                reply[5] = memmap[addr*2+3];
                reply[6] = memmap[addr*2+0];
                reply[7] = memmap[addr*2+1];
                reply[8] = 0;
            }
        }
    }

    ret = sscanf(reply, "%x", &regvalue);
    if (ret != 1) {
        return -1;
    } else {
        return regvalue;
    }

}


// read just a short part from the rover's memmap, and update it in the local memmap copy
int rover_read_register(unsigned char controller_addr, unsigned char register_addr, unsigned char length, unsigned char *memmap, struct roverstruct *rover) {
    unsigned char datatosend[64];
    unsigned char reply[256];
    int datalen, recvbytes, rs485err=0xFF, readeyerr=1;

    if ( (length != 1) && (length != 2) && (length != 4) && (length != 64) ) {
        printf("rover_read_register(controller_addr:0x%x, register_addr:0x%x): Invalid length value (%d)! Valid length values are: 1, 2, 4, 64.\n", controller_addr, register_addr, length);
        return -1;
    }

    bzero(reply, 256);
    datalen = sprintf(datatosend, "r%02X %02X %02X\n", controller_addr, register_addr, length);
    recvbytes = send_command_raw(datatosend, datalen, reply);
    if (recvbytes == 0) {
        return -1;
    }

    while (rs485err != 0) {
        rs485err = check_and_remove_rs485_error(reply);
        if (rs485err < 0) {
            printf("Unknown error message ?: %s\n", reply);
            return -2;
        }
        if (rs485err == 0x10) {
            rover->rs485_err_0x10 += 1;
        } else {
            if (rs485err == 0x1F) {
                rover->rs485_err_0x1F += 1;
            }
        }
    }

    recvbytes = strlen(reply);

    while (readeyerr != 0) {
        readeyerr = check_and_remove_readey(reply);
        if (readeyerr < 0) {
            printf("Unknown message when looking for 'readey' ?: %s\n", reply);
            return -2;
        }
    }

    if (check_invalidchars(reply) != 0) {
        printf("Unknown character in reply: %s\n", reply);
        return -2;
    }

    // probably it was only an error msg (filtered by rs485err or readey)
    if (recvbytes == 0) {
        return -1;
    }

    if ( (recvbytes != (2 * length + 2)) && (reply[2*length] != '\r') && (reply[2*length+1] != '\n') ) {
        printf("Invalid response to command(len:%d)/register_addr 0x%X recvbytes: %d msg: %s\n", length, register_addr, recvbytes, reply);
        return -1;
    }
    memcpy(&memmap[register_addr*2], reply, 2 * length);

    return 0;

}

int rover_write_register_uint8(unsigned char controller_addr, unsigned char register_addr, unsigned char data, unsigned char *reply) {
        unsigned char datatosend[32];
        int datalen;

        datalen = sprintf(datatosend, "w%02X %02X %02X\n", controller_addr, register_addr, data);
#ifdef DEBUG
        printf("Write to rover uint8: len:%d data:%s\n", datalen, datatosend);
#endif

        return send_command_raw(datatosend, datalen, reply);
}


int rover_write_register_uint16(unsigned char controller_addr, unsigned char register_addr, unsigned int data, unsigned char *reply) {
        unsigned char datatosend[64];
        int datalen;

        if (data < 0) { data = 0; }
        else { if (data > 65535) { data = 65535; } }
        datalen = sprintf(datatosend, "w%02X %02X %02X%02X\n", controller_addr, register_addr, data & 0xFF, data >> 8);
#ifdef DEBUG
        printf("Write to rover uint16: len:%d data:%s\n", datalen, datatosend);
#endif

        return send_command_raw(datatosend, datalen, reply);
}


int rover_write_register_uint32(unsigned char controller_addr, unsigned char register_addr, unsigned int data, unsigned char *reply) {
        unsigned char datatosend[64];
        int datalen;

        datalen = sprintf(datatosend, "w%02X %02X %02X%02X%02X%02X\n", controller_addr, register_addr, data & 0xFF, (data & 0xFF00) >> 8, (data & 0xFF0000) >> 16, (data & 0xFF000000) >> 24);
#ifdef DEBUG
        printf("Write to rover uint32: len:%d data:%s\n", datalen, datatosend);
#endif

        return send_command_raw(datatosend, datalen, reply);
}


int rover_write_register_triple_zero_uint16(unsigned char controller_addr, unsigned char register_addr, unsigned char *reply) {
        unsigned char datatosend[64];
        int datalen;

        datalen = sprintf(datatosend, "w%02X %02X 000000000000\n", controller_addr, register_addr);
#ifdef DEBUG
        printf("Write to rover triple_zero_uint16: len:%d data:%s\n", datalen, datatosend);
#endif

        return send_command_raw(datatosend, datalen, reply);
}


int rover_write_register_int16(unsigned char controller_addr, unsigned char register_addr, int data, unsigned char *reply) {
        unsigned char datatosend[64];
        int datalen;

        if (data < 0) { data = 0xFFFF - abs(data); }
        datalen = sprintf(datatosend, "w%02X %02X %02X%02X\n", controller_addr, register_addr, data & 0xFF, data >> 8);
#ifdef DEBUG
        printf("Write to rover int16: len:%d data:%s\n", datalen, datatosend);
#endif

        return send_command_raw(datatosend, datalen, reply);
}


unsigned int rover_get_controller_addr(struct roverstruct *rover, unsigned int controller_id) {

    unsigned char controller_addr;

    switch (controller_id) {
        case 1:
            controller_addr = rover->regs->controller_addr_main;
            break;
        case 2:
            controller_addr = rover->regs->controller_addr_second;
            break;
        case 0:
        default:
            controller_addr = ROVER_CONTROLLER_ADDR_MAIN;
    }

    return controller_addr;
}


int rover_read_full_memmap(unsigned char *memmap, unsigned int controller_addr, struct roverstruct *rover) {
    int ret, datalen, i;
    unsigned char replyfull[512];
    unsigned char datatosend[32];
    unsigned char offset = 0x00;

    for (ret = 0; ret < 512; ret++) {
        replyfull[ret] = 'X';
    }
    replyfull[511] = 0;

    for (offset = 0x00; offset <= 0x80; offset += 0x40) {
        // try 3 times
        for (i = 0; i < 3; i++) {
            ret = rover_read_register(controller_addr, offset, 64, replyfull, rover);
            if (ret ==  0) { break; }
            if (ret == -2) { return -2; }
        }
        offset += 0x40;
    }

    for (ret = 0; replyfull[ret] != 'X'; ret++) { memmap[ret] = replyfull[ret]; }

    return ret;

}


unsigned char rover_identify(struct roverstruct *rover) {

    int ret;

    ret = rover_read_full_memmap(rover->memmap_main, 0, rover);

    rover->sysname = rover_get_sysname(rover->memmap_main);
    rover->firmrev = rover_get_firmrev(rover->memmap_main);

    switch (rover->sysname) {
        case 0x21:
            strcpy(rover->fullname, "MecanumRover V2.1\0");
            rover->has_second_controller = 1;
            rover->has_Y_speed = 1;
            rover->motor_count = 4;
            rover->regs = (struct rover_regs *)&rover_regs_mecanumrover21;
            return 0;

        case 0x30:
            strcpy(rover->fullname, "MegaRover V3\0");
            rover->has_second_controller = 0;
            rover->has_Y_speed = 0;
            rover->motor_count = 2;
            rover->regs = (struct rover_regs *)&rover_regs_megarover3;
            return 0;

        default:
            strcpy(rover->fullname, "UNKNOWN\0");
            rover->regs = (struct rover_regs *)&rover_regs_unknown;
            return 1;
    }
}


// extract values from previously read memmap
int    rover_get_sysname(unsigned char *memmap)            { return read_register_from_memmap(memmap, ROVER_REG_SYSTEMNAME, 2); }
int    rover_get_firmrev(unsigned char *memmap)            { return read_register_from_memmap(memmap, ROVER_REG_FIRMWAREREVISION, 2); }
double rover_get_uptime(unsigned char *memmap)             { return read_register_from_memmap(memmap, ROVER_REG_UPTIME, 4)/1000.0; }

unsigned char rover_get_motor_status(unsigned char *memmap)       { return read_register_from_memmap(memmap, ROVER_REG_ENABLEMOTORS, 1); }

int    rover_get_outputoffset0(unsigned char *memmap)      { return conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_OUTPUTOFFSET0, 2)); }
int    rover_get_outputoffset1(unsigned char *memmap)      { return conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_OUTPUTOFFSET1, 2)); }
double rover_get_max_current0(unsigned char *memmap)       { return (read_register_from_memmap(memmap, ROVER_REG_MAXCURRENT0, 2)/4096.0)*11.58; } // 0x1000 = 11.58A
double rover_get_max_current1(unsigned char *memmap)       { return (read_register_from_memmap(memmap, ROVER_REG_MAXCURRENT1, 2)/4096.0)*11.58; } // 0x1000 = 11.58A
double rover_get_current_limit0(unsigned char *memmap)     { return (read_register_from_memmap(memmap, ROVER_REG_CURRENTLIMIT0, 2)/4096.0)*11.58; } // 0x1000 = 11.58A
double rover_get_current_limit1(unsigned char *memmap)     { return (read_register_from_memmap(memmap, ROVER_REG_CURRENTLIMIT1, 2)/4096.0)*11.58; } // 0x1000 = 11.58A
int    rover_get_measured_position0(unsigned char *memmap) { return read_register_from_memmap(memmap, ROVER_REG_MEASUREDPOS0, 4); }
int    rover_get_measured_position1(unsigned char *memmap) { return read_register_from_memmap(memmap, ROVER_REG_MEASUREDPOS1, 4); }
int    rover_get_speed0(unsigned char *memmap)             { return conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_SPEED0, 2)); }
int    rover_get_speed1(unsigned char *memmap)             { return conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_SPEED1, 2)); }

double rover_get_motoroutput_calc0(unsigned char *memmap)  { return (conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_MOTOROUTPUTCALC0, 2))/4096.0)*100; } // 100% = 0x1000
double rover_get_motoroutput_calc1(unsigned char *memmap)  { return (conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_MOTOROUTPUTCALC1, 2))/4096.0)*100; } // 100% = 0x1000

double rover_get_battery_voltage(unsigned char *memmap)    { return (read_register_from_memmap(memmap, ROVER_REG_BATTERYVOLTAGE, 2)/4095.0)*29.7; } // 29.7V = 0x0FFF
int    rover_get_encoder_value0(unsigned char *memmap)     { return read_register_from_memmap(memmap, ROVER_REG_ENCODERVALUE0, 4); }
int    rover_get_encoder_value1(unsigned char *memmap)     { return read_register_from_memmap(memmap, ROVER_REG_ENCODERVALUE1, 4); }

double rover_get_measured_current_value0(unsigned char *memmap) { return (read_register_from_memmap(memmap, ROVER_REG_MEASUREDCURRENT0, 2)/4096.0)*11.58; } // 0x1000 = 11.58A
double rover_get_measured_current_value1(unsigned char *memmap) { return (read_register_from_memmap(memmap, ROVER_REG_MEASUREDCURRENT1, 2)/4096.0)*11.58; } // 0x1000 = 11.58A

int    rover_get_X_speed(unsigned char *memmap)        { return conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_SPEED_X, 2)); }
int    rover_get_Y_speed(unsigned char *memmap)        { return conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_SPEED_Y, 2)); }
int    rover_get_rotation_speed(unsigned char *memmap) { return conv_int16_to_int32(read_register_from_memmap(memmap, ROVER_REG_ROTATION, 2)); }

// write commands
int rover_enable_motors_main(unsigned char *reply)   { return rover_write_register_uint8(ROVER_CONTROLLER_ADDR_REAR,  ROVER_REG_ENABLEMOTORS, 3, reply); }
int rover_enable_motors_front(unsigned char *reply)  { return rover_write_register_uint8(ROVER_CONTROLLER_ADDR_FRONT, ROVER_REG_ENABLEMOTORS, 3, reply); }
int rover_disable_motors_main(unsigned char *reply)  { return rover_write_register_uint8(ROVER_CONTROLLER_ADDR_REAR,  ROVER_REG_ENABLEMOTORS, 0, reply); }
int rover_disable_motors_front(unsigned char *reply) { return rover_write_register_uint8(ROVER_CONTROLLER_ADDR_FRONT, ROVER_REG_ENABLEMOTORS, 0, reply); }

int rover_set_X_speed(int speed, unsigned char *reply)        { return rover_write_register_int16(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_SPEED_X, speed, reply); }
int rover_set_Y_speed(int speed, unsigned char *reply)        { return rover_write_register_int16(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_SPEED_Y, speed, reply); }
int rover_set_rotation_speed(int speed, unsigned char *reply) { return rover_write_register_int16(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_ROTATION, speed, reply); }
int rover_set_XYrotation_speed_to_zero(unsigned char *reply)  { return rover_write_register_triple_zero_uint16(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_SPEED_X, reply); }

int rover_set_encoder_value0(unsigned int value, unsigned char *reply)      { return rover_write_register_uint32(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_ENCODERVALUE0, value, reply); }
int rover_set_encoder_value1(unsigned int value, unsigned char *reply)      { return rover_write_register_uint32(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_ENCODERVALUE1, value, reply); }
int rover_set_measured_position0(unsigned int value, unsigned char *reply)  { return rover_write_register_uint32(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_MEASUREDPOS0, value, reply); }
int rover_set_measured_position1(unsigned int value, unsigned char *reply)  { return rover_write_register_uint32(ROVER_CONTROLLER_ADDR_REAR, ROVER_REG_MEASUREDPOS1, value, reply); }

/*
 "kkk" commands:

 We developed another command set (in a way not to interfere with the original protocol) for a more robust communication method,
 where everything is sent three times, this way error detection and correction can be both simply achieved.
 The robot controller's firmware was modified accordingly...
 As the source code of the robot controller's firmware is of course not open/free, we cannot publish it here.
 In case you have bought the robot and have the source code of the firmware, we can send you the patches on your request.
 Also, to increase safety, kkk set speed command always has a timeout, so it should be periodically repeated, otherwise the robot will stop.
*/

// kkk STOP command - "STPSTPSTP"
int rover_kset_STOP(unsigned char *reply) {
    unsigned char datatosend[16] = "STPSTPSTP\n\n\n\0";
#ifdef DEBUG
    printf("Write to rover kSTOP: %s\n", datatosend);
#endif
    return send_command_raw(datatosend, 12, reply);
}


// kkk set speeds command
int rover_kset_XYrotation_speed(int xspeed, int yspeed, int rotspeed, unsigned char *reply) {

    unsigned char datatosend[32];
    int datalen;
    unsigned char xspeed_hi, yspeed_hi, rotspeed_hi;
    unsigned char xspeed_lo, yspeed_lo, rotspeed_lo;

    if (  xspeed >  LIMIT_SPEED_X)   { xspeed   =  LIMIT_SPEED_X;   }
    if (  yspeed >  LIMIT_SPEED_Y)   { yspeed   =  LIMIT_SPEED_Y;   }
    if (rotspeed >  LIMIT_SPEED_ROT) { rotspeed =  LIMIT_SPEED_ROT; }
    if (  xspeed < -LIMIT_SPEED_X)   { xspeed   = -LIMIT_SPEED_X;   }
    if (  yspeed < -LIMIT_SPEED_Y)   { yspeed   = -LIMIT_SPEED_Y;   }
    if (rotspeed < -LIMIT_SPEED_ROT) { rotspeed = -LIMIT_SPEED_ROT; }

    rotspeed/=4;

    if (xspeed   < 0) { xspeed   = 0xFFFF - abs(xspeed);   }
    if (yspeed   < 0) { yspeed   = 0xFFFF - abs(yspeed);   }
    if (rotspeed < 0) { rotspeed = 0xFFFF - abs(rotspeed); }

    xspeed_hi   = (  xspeed >> 8) & 0xFF;
    yspeed_hi   = (  yspeed >> 8) & 0xFF;
    rotspeed_hi = (rotspeed >> 8) & 0xFF;
    xspeed_lo   =   xspeed & 0xFF;
    yspeed_lo   =   yspeed & 0xFF;
    rotspeed_lo = rotspeed & 0xFF;

    // add +1 so no 0x00 s are sent (original proto is string based where 0x00 means end-of-string)
    // but be careful about negative values (so only under <127 for hi)
    if (xspeed_hi   < 127) { xspeed_hi++; };
    if (yspeed_hi   < 127) { yspeed_hi++; };
    if (rotspeed_hi < 127) { rotspeed_hi++; };

    if (xspeed_lo   < 255) { xspeed_lo++; };
    if (yspeed_lo   < 255) { yspeed_lo++; };
    if (rotspeed_lo < 255) { rotspeed_lo++; };

    // should not happen (because of the -1500 to +1500 interval), just to be on the safe side
    if ( (xspeed_hi   == 0x0A) || (xspeed_hi   == 0x72) || (xspeed_hi   == 0x52) || (xspeed_hi   == 0x77) || (xspeed_hi   == 0x57) ) { xspeed_hi--; }
    if ( (yspeed_hi   == 0x0A) || (yspeed_hi   == 0x72) || (yspeed_hi   == 0x52) || (yspeed_hi   == 0x77) || (yspeed_hi   == 0x57) ) { yspeed_hi--; }
    if ( (rotspeed_hi == 0x0A) || (rotspeed_hi == 0x72) || (rotspeed_hi == 0x52) || (rotspeed_hi == 0x77) || (rotspeed_hi == 0x57) ) { rotspeed_hi--; }

    // this could happen, so let's change 0x0A (newline) to 0x0A-1 and also 'r' 'R' 'w' 'W'
    // orig speed will be changed but a reduced speed is still better than increased speed
    if ( (xspeed_lo   == 0x0A) || (xspeed_lo   == 0x72) || (xspeed_lo   == 0x52) || (xspeed_lo   == 0x77) || (xspeed_lo   == 0x57) ) { xspeed_lo--; }
    if ( (yspeed_lo   == 0x0A) || (yspeed_lo   == 0x72) || (yspeed_lo   == 0x52) || (yspeed_lo   == 0x77) || (yspeed_lo   == 0x57) ) { yspeed_lo--; }
    if ( (rotspeed_lo == 0x0A) || (rotspeed_lo == 0x72) || (rotspeed_lo == 0x52) || (rotspeed_lo == 0x77) || (rotspeed_lo == 0x57) ) { rotspeed_lo--; }

    datatosend[ 0] = 'k';
    datatosend[ 1] = 'k';
    datatosend[ 2] = 'k';

    datatosend[ 3] = xspeed_hi;
    datatosend[ 4] = xspeed_lo;
    datatosend[ 5] = yspeed_hi;
    datatosend[ 6] = yspeed_lo;
    datatosend[ 7] = rotspeed_hi;
    datatosend[ 8] = rotspeed_lo;

    datatosend[ 9] = xspeed_hi;
    datatosend[10] = xspeed_lo;
    datatosend[11] = yspeed_hi;
    datatosend[12] = yspeed_lo;
    datatosend[13] = rotspeed_hi;
    datatosend[14] = rotspeed_lo;

    datatosend[15] = xspeed_hi;
    datatosend[16] = xspeed_lo;
    datatosend[17] = yspeed_hi;
    datatosend[18] = yspeed_lo;
    datatosend[19] = rotspeed_hi;
    datatosend[20] = rotspeed_lo;

    datatosend[21] = 0x0A;
    datatosend[22] = 0x0A;
    datatosend[23] = 0x0A;

    datatosend[24] = 0;

    datalen = 24;
#ifdef DEBUG
    printf("Write to rover k: len:%d data:%s\n", datalen, datatosend);
#endif

    return send_command_raw(datatosend, datalen, reply);
}
