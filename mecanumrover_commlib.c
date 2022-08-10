/*
    NLAB-MecanumCommlib for Linux, a simple library to control VStone MecanumRover 2.1 / VStone MegaRover 3
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021-2022
    version 0.60
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


// { has_second_controller, has_Y_speed, motor_count, enablemotors_on, enablemotors_off }
const struct rover_config rover_config_unknown        = { 0, 0, 2, 3, 0 };
const struct rover_config rover_config_mecanumrover21 = { 1, 1, 4, 3, 0 };
const struct rover_config rover_config_megarover3     = { 0, 0, 2, 3, 0 };


const struct rover_regs rover_regs_unknown = {
    CONTROLLER_ADDR_MAIN, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    ROVER_REG_SYSTEMNAME, ROVER_REG_FIRMWAREREVISION, ROVER_REG_UPTIME,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};

const struct rover_regs rover_regs_mecanumrover21 = {
    CONTROLLER_ADDR_MAIN, CONTROLLER_ADDR_SECOND,
    MECANUMROVER21_REG_BATTERYVOLTAGE,
    MECANUMROVER21_REG_SPEED_X, MECANUMROVER21_REG_SPEED_Y, MECANUMROVER21_REG_ROTATION, ROVER_REG_ENABLEMOTORS,
    ROVER_REG_SYSTEMNAME, ROVER_REG_FIRMWAREREVISION, ROVER_REG_UPTIME,
    MECANUMROVER21_REG_OUTPUTOFFSET0, MECANUMROVER21_REG_OUTPUTOFFSET1,
    MECANUMROVER21_REG_MOTOROUTPUTCALC0, MECANUMROVER21_REG_MOTOROUTPUTCALC1,
    MECANUMROVER21_REG_MAXCURRENT0, MECANUMROVER21_REG_MAXCURRENT1,
    MECANUMROVER21_REG_CURRENTLIMIT0, MECANUMROVER21_REG_CURRENTLIMIT1,
    MECANUMROVER21_REG_MEASUREDCURRENT0, MECANUMROVER21_REG_MEASUREDCURRENT1,
    MECANUMROVER21_REG_MEASUREDPOS0, MECANUMROVER21_REG_MEASUREDPOS1,
    MECANUMROVER21_REG_SPEED0, MECANUMROVER21_REG_SPEED1,
    MECANUMROVER21_REG_ENCODERVALUE0, MECANUMROVER21_REG_ENCODERVALUE1,
    0x00, 0x00
};

const struct rover_regs rover_regs_megarover3 = {
    CONTROLLER_ADDR_MAIN, CONTROLLER_ADDR_MAIN,
    MEGAROVER3_REG_BATTERYVOLTAGE,
    MEGAROVER3_REG_SPEED_X, MEGAROVER3_REG_SPEED_Y, MEGAROVER3_REG_ROTATION, ROVER_REG_ENABLEMOTORS,
    ROVER_REG_SYSTEMNAME, ROVER_REG_FIRMWAREREVISION, ROVER_REG_UPTIME,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    MEGAROVER3_REG_MEASUREDCURRENT0, MEGAROVER3_REG_MEASUREDCURRENT1,
    0x00, 0x00,
    0x00, 0x00,
    MEGAROVER3_REG_ENCODERVALUE0, MEGAROVER3_REG_ENCODERVALUE1,
    MEGAROVER3_REG_MOTORSPEED0, MEGAROVER3_REG_MOTORSPEED1,
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


int check_serial_dev() {
    int chkfd;

    chkfd = open(DEVFILE, O_RDWR);
    if (chkfd == -1) {
        return -1;
    }

    close(chkfd);

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
    tv.tv_usec = REPLYWAIT_TIMEOUT_USEC;

    ret = write(serial, message, ++messagelen);
    if (ret != messagelen) {
        printf("write() returned fewer bytes than expected!\n");
        if (ret == -1) {
            perror("write(serial): ");
        }
    }

    datareceived = 0;

    // if reply is NULL, then we do not care about reply
    if (reply != NULL) {

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
                    tv.tv_usec = REPLYWAIT_TIMEOUT_USEC;
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
                    printf("No reply within " REPLYWAIT_TIMEOUT_USEC " usec.\n");
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

        reply[datareceived] = 0;

    }

    close(serial);

    return datareceived;

}


// hexstring->endianness->num
int read_register_from_memmap(unsigned char *memmap, unsigned char register_addr, unsigned char register_length) {

    unsigned char reply[16];
    int register_value, ret;

    if ( (register_length != 1) && (register_length != 2) && (register_length != 4) ) {
        printf("read_register_from_memmap(register_addr:0x%x): Invalid length value (%d)! Valid register_length values are: 1, 2, 4.\n", register_addr, register_length);
        return -1; // -1 is ok here, because this is the programmer's mistake...
    }

    switch (register_length) {

        case 1:
            reply[0] = memmap[register_addr*2+0];
            reply[1] = memmap[register_addr*2+1];
            reply[2] = 0;
            break;

        case 2:
            reply[0] = memmap[register_addr*2+2];
            reply[1] = memmap[register_addr*2+3];
            reply[2] = memmap[register_addr*2+0];
            reply[3] = memmap[register_addr*2+1];
            reply[4] = 0;
            break;

        case 4:
            reply[0] = memmap[register_addr*2+6];
            reply[1] = memmap[register_addr*2+7];
            reply[2] = memmap[register_addr*2+4];
            reply[3] = memmap[register_addr*2+5];
            reply[4] = memmap[register_addr*2+2];
            reply[5] = memmap[register_addr*2+3];
            reply[6] = memmap[register_addr*2+0];
            reply[7] = memmap[register_addr*2+1];
            reply[8] = 0;
            break;
    }

    ret = sscanf(reply, "%x", &register_value);
    if (ret != 1) {
        return -1;
    } else {
        return register_value;
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
            controller_addr = CONTROLLER_ADDR_DEFAULT;
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
    }

    for (ret = 0; replyfull[ret] != 'X'; ret++) { memmap[ret] = replyfull[ret]; }

    return ret;

}


unsigned char rover_identify(struct roverstruct *rover) {

    int ret;

    ret = rover_read_full_memmap(rover->memmap_main, CONTROLLER_ADDR_DEFAULT, rover);
    if (ret < 0) {
        return ret;
    } else {
        return rover_identify_from_main_memmap(rover);
    }
}


unsigned char rover_identify_from_main_memmap(struct roverstruct *rover) {

    rover->config = (struct rover_config *)&rover_config_unknown;
    rover->regs = (struct rover_regs *)&rover_regs_unknown;
    rover->sysname = rover_get_sysname(rover);
    rover->firmrev = rover_get_firmrev(rover);
    rover->rs485_err_0x10 = 0;
    rover->rs485_err_0x1F = 0;

    switch (rover->sysname) {
        case SYSNAME_MECANUMROVER21:
            strcpy(rover->fullname, "MecanumRover V2.1\0");
            rover->config = (struct rover_config *)&rover_config_mecanumrover21;
            rover->regs = (struct rover_regs *)&rover_regs_mecanumrover21;
            return 0;

        case SYSNAME_MEGAROVER3:
            strcpy(rover->fullname, "MegaRover V3\0");
            rover->config = (struct rover_config *)&rover_config_megarover3;
            rover->regs = (struct rover_regs *)&rover_regs_megarover3;
            return 0;

        default:
            strcpy(rover->fullname, "UNKNOWN\0");
            return 1;
    }
}


// extract values from previously read memmap
int    rover_get_sysname(struct roverstruct *rover)         { return read_register_from_memmap(rover->memmap_main, rover->regs->systemname, 2); }
int    rover_get_firmrev(struct roverstruct *rover)         { return read_register_from_memmap(rover->memmap_main, rover->regs->firmwarerevision, 2); }
double rover_get_uptime(struct roverstruct *rover)          { return read_register_from_memmap(rover->memmap_main, rover->regs->uptime, 4) / 1000.0; }
double rover_get_battery_voltage(struct roverstruct *rover) { return (read_register_from_memmap(rover->memmap_main, rover->regs->batteryvoltage, 2) / 4095.0) * 29.7; } // 29.7V = 0x0FFF
int    rover_get_X_speed(struct roverstruct *rover)         { return conv_int16_to_int32(read_register_from_memmap(rover->memmap_main, rover->regs->speed_x, 2)); }
int    rover_get_Y_speed(struct roverstruct *rover)         { return conv_int16_to_int32(read_register_from_memmap(rover->memmap_main, rover->regs->speed_y, 2)); }
int    rover_get_rotation_speed(struct roverstruct *rover)  { return conv_int16_to_int32(read_register_from_memmap(rover->memmap_main, rover->regs->rotation, 2)); }

unsigned char rover_get_motor_status(struct roverstruct *rover, unsigned char *memmap)     { return read_register_from_memmap(memmap, rover->regs->enablemotors, 1); }
int    rover_get_outputoffset0(struct roverstruct *rover, unsigned char *memmap)           { return conv_int16_to_int32(read_register_from_memmap(memmap, rover->regs->outputoffset0, 2)); }
int    rover_get_outputoffset1(struct roverstruct *rover, unsigned char *memmap)           { return conv_int16_to_int32(read_register_from_memmap(memmap, rover->regs->outputoffset1, 2)); }
double rover_get_max_current0(struct roverstruct *rover, unsigned char *memmap)            { return (read_register_from_memmap(memmap, rover->regs->maxcurrent0, 2)/ 4096.0) * 11.58; } // 0x1000 = 11.58A
double rover_get_max_current1(struct roverstruct *rover, unsigned char *memmap)            { return (read_register_from_memmap(memmap, rover->regs->maxcurrent1, 2)/ 4096.0) * 11.58; } // 0x1000 = 11.58A
double rover_get_current_limit0(struct roverstruct *rover, unsigned char *memmap)          { return (read_register_from_memmap(memmap, rover->regs->currentlimit0, 2)/ 4096.0) * 11.58; } // 0x1000 = 11.58A
double rover_get_current_limit1(struct roverstruct *rover, unsigned char *memmap)          { return (read_register_from_memmap(memmap, rover->regs->currentlimit1, 2)/ 4096.0) * 11.58; } // 0x1000 = 11.58A
int    rover_get_measured_position0(struct roverstruct *rover, unsigned char *memmap)      { return read_register_from_memmap(memmap, rover->regs->measuredpos0, 4); }
int    rover_get_measured_position1(struct roverstruct *rover, unsigned char *memmap)      { return read_register_from_memmap(memmap, rover->regs->measuredpos1, 4); }
int    rover_get_speed0(struct roverstruct *rover, unsigned char *memmap)                  { return conv_int16_to_int32(read_register_from_memmap(memmap, rover->regs->speed0, 2)); }
int    rover_get_speed1(struct roverstruct *rover, unsigned char *memmap)                  { return conv_int16_to_int32(read_register_from_memmap(memmap, rover->regs->speed1, 2)); }
int    rover_get_motorspeed0(struct roverstruct *rover, unsigned char *memmap)             { return read_register_from_memmap(memmap, rover->regs->motorspeed0, 4); }
int    rover_get_motorspeed1(struct roverstruct *rover, unsigned char *memmap)             { return read_register_from_memmap(memmap, rover->regs->motorspeed1, 4); }
double rover_get_motoroutput_calc0(struct roverstruct *rover, unsigned char *memmap)       { return (conv_int16_to_int32(read_register_from_memmap(memmap, rover->regs->motoroutputcalc0, 2)) / 4096.0) * 100; } // 100% = 0x1000
double rover_get_motoroutput_calc1(struct roverstruct *rover, unsigned char *memmap)       { return (conv_int16_to_int32(read_register_from_memmap(memmap, rover->regs->motoroutputcalc1, 2)) / 4096.0) * 100; } // 100% = 0x1000
int    rover_get_encoder_value0(struct roverstruct *rover, unsigned char *memmap)          { return read_register_from_memmap(memmap, rover->regs->encodervalue0, 4); }
int    rover_get_encoder_value1(struct roverstruct *rover, unsigned char *memmap)          { return read_register_from_memmap(memmap, rover->regs->encodervalue1, 4); }
double rover_get_measured_current_value0(struct roverstruct *rover, unsigned char *memmap) { return (read_register_from_memmap(memmap, rover->regs->measuredcurrent0, 2) / 4096.0) * 11.58; } // 0x1000 = 11.58A
double rover_get_measured_current_value1(struct roverstruct *rover, unsigned char *memmap) { return (read_register_from_memmap(memmap, rover->regs->measuredcurrent1, 2) / 4096.0) * 11.58; } // 0x1000 = 11.58A


// write commands
int rover_enable_motors( struct roverstruct *rover, unsigned char controller_addr, unsigned char *reply) { return rover_write_register_uint8(controller_addr, rover->regs->enablemotors, rover->config->enablemotors_on,  reply); }
int rover_disable_motors(struct roverstruct *rover, unsigned char controller_addr, unsigned char *reply) { return rover_write_register_uint8(controller_addr, rover->regs->enablemotors, rover->config->enablemotors_off, reply); }

int rover_set_X_speed(struct roverstruct *rover, int speed_x, unsigned char *reply)          { return rover_write_register_int16(rover->regs->controller_addr_main, rover->regs->speed_x, speed_x, reply); }
int rover_set_Y_speed(struct roverstruct *rover, int speed_y, unsigned char *reply)          { return rover_write_register_int16(rover->regs->controller_addr_main, rover->regs->speed_y, speed_y, reply); }
int rover_set_rotation_speed(struct roverstruct *rover, int speed_rot, unsigned char *reply) { return rover_write_register_int16(rover->regs->controller_addr_main, rover->regs->rotation, speed_rot, reply); }
int rover_set_XYrotation_speed_to_zero(struct roverstruct *rover, unsigned char *reply)      { return rover_write_register_triple_zero_uint16(rover->regs->controller_addr_main, rover->regs->speed_x, reply); }

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

    rotspeed /= 4;

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
