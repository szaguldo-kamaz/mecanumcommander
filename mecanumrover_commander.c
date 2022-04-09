/*
    NLAB-MecanumCommander for Linux, a simple bridge to control VStone MecanumRover 2.1
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021
    version 0.40
    https://github.com/szaguldo-kamaz/
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ncurses.h>
#include <locale.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/timeb.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include "mecanumrover_commlib.h"

#define COMMANDER_VERSION  "0.40"
#define COMMANDER_PASSWORD "PASSWORD"

#define REPEAT_TIME_SEC_CMDSENT 0.4
#define REPEAT_TIME_SEC_KCMDSENT 0.4
#define REPEAT_TIME_SEC_REMOTECMDRECV 0.5
//#define REPEAT_TIME_SEC_MEMMAPREAD 0.5
#define REPEAT_TIME_SEC_MEMMAPREAD 0.4

int got_sigpipe = 0;


void errormsg(unsigned char *errmsg, unsigned char xpos) {
    attron(COLOR_PAIR(5));
    mvprintw(1, xpos, errmsg);
    attroff(COLOR_PAIR(5));
    beep();
    timeout(-1);
    getch();
}


void sigpipe_handler(int signum) {
    got_sigpipe = 1;
}


void stoprobot(char usekcommands, char *answer) {
    if (usekcommands == 1) {
        rover_kset_STOP(answer);
    } else {
        rover_set_XYrotation_speed_to_zero(answer);
    }
}


void commandsend_lamp_on() {
    attron(COLOR_PAIR(7));
//    mvprintw(2 + 5, 2 + 9, "⚞ ⚟");
    mvprintw(2, 2 + 23, "⚟");
    attroff(COLOR_PAIR(7));
    refresh();
}


void commandsend_lamp_off() {
    attron(COLOR_PAIR(1));
//    mvprintw(2 + 5, 2 + 9, "     ");
    mvprintw(2, 2 + 23, "  ");
    attroff(COLOR_PAIR(1));
    refresh();
}


int main() {

    int ret;
    unsigned char answer[BUFFER_SIZE];

    struct roverstruct rover;

    struct timeb timestruct;
    double time_current, time_last_memmapread, time_last_cmdsent, time_last_kcmdsent, time_last_remotecmd_recv;
    unsigned char remotecmd_timed_out=1;

    int speedX=0, speedY=0, rotate=0;
    int prevspeedX=0, prevspeedY=0, prevrotate=0;

    char quit=0, c, c2, c3, c4, c5, c6;
    int roverdrawx=5, roverdrawy=10;
    int commanddrawx=2, commanddrawy=2;
    int statusdrawx=30, statusdrawy=2;
    int aboutdrawx=2, aboutdrawy=16;

    fd_set commfdset;
    struct timeval tv;

    int listenfd=0, clientfd=0, sockread=0, socketcommbuff_offset=0;
    struct sockaddr_in serv_addr;
    unsigned char socketcommbuff[1025];
    unsigned char receivedcommand[16];
    unsigned char set_new_spx_value_from_remote=0;
    unsigned char set_new_spy_value_from_remote=0;
    unsigned char set_new_rot_value_from_remote=0;

    unsigned char remotecontrol=0;      // set to 1 to listen on tcp/3475 for easy remote control
    unsigned char dummymode=0;          // for testing, do not send real commands to the robot
    unsigned char keyboardmode=1;       // for future use...
    unsigned char repeatcommands=1;     // repeat commands every REPEAT_TIME_SEC_CMDSENT, so "commandtimeout" on the robot's controller won't trigger
    unsigned char usekcommands=0;       // use the "triple command set"
    unsigned char readmemmapfromfile=1; // do not get the memmap from the robot, instead read it from a file

    rover.rs485_err_0x10 = 0;
    rover.rs485_err_0x1F = 0;

    if (dummymode == 1) {
        int fd;

        fd = open("memmap_sample.dat", O_RDONLY);
        ret = read(fd, rover.memmap_main, 386);
        ret = read(fd, rover.memmap_front, 386);
        close(fd);
    } else {
        ret = rover_read_register(ROVER_CONTROLLER_ADDR_REAR, 0x00, 4, rover.memmap_main, &rover); // get only the sysname + firmrev (first 4 bytes of memmap)
        if (ret == -2) {
            printf("Fatal error!\n");
            exit(1);
        }
        if (ret == -1) { // wait and try again once
            usleep(1000000);
            ret = rover_read_register(ROVER_CONTROLLER_ADDR_REAR, 0x00, 4, rover.memmap_main, &rover);
        }
        if (ret != 0) {
            printf("Invalid or no reply from rover! Can't get ID! Fatal error!\n");
            exit(1);
        }
    }

    if (rover_identify(&rover) == 0) {
        printf("Rover found: 0x%x:%s FWRev: 0x%x\n", rover.sysname, rover.fullname, rover.firmrev);
    } else {
        printf("Unknown rover type: 0x%X!\n", rover.sysname);
        exit(1);
    }

    // bind to tcp/3475
    if (remotecontrol == 1) {
        typedef void (*sighandler_t)(int);
        sighandler_t sigret;
        unsigned char replymsg[128];
        int replylen, wret;
        int enable=1;

        listenfd = socket(AF_INET, SOCK_STREAM, 0);
        if (listenfd == -1) {
            perror("socket()");
            exit(3);
        }
        sigret = signal(SIGPIPE, sigpipe_handler);
        if (sigret == SIG_ERR) {
            perror("signal(SIGPIPE)");
            exit(3);
        }

        if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
            perror("setsockopt(SO_REUSEADDR)");
            exit(3);
        }

        bzero(&serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        serv_addr.sin_port = htons(3475);
        ret = bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
        if (ret == -1) {
            perror("bind(INADDR_ANY/tcp/3475)");
            exit(3);
        }
        listen(listenfd, 1);
        if (ret == -1) {
            perror("listen()");
            exit(3);
        }
        printf("Waiting for connection on tcp/3475...\n");
        clientfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
        if (clientfd == -1) {
            perror("accept()");
            exit(3);
        }
        replylen = sprintf(replymsg, "I'm NLAB-MecanumCommander. Please authenticate yourself.\r\n");
        wret = write(clientfd, replymsg, replylen);
        if (wret == -1) {
            printf("Cannot send reply to client! Connection lost?\n");
            perror("write()");
            exit(6);
        }

        FD_ZERO(&commfdset);
        FD_SET(clientfd, &commfdset);
        tv.tv_sec = 10;
        tv.tv_usec = 0;
        ret = select(clientfd + 1, &commfdset, NULL, NULL, &tv);
        if (ret == -1) {
            perror("select()");
            exit(7);
        }
        if (got_sigpipe == 1) {
            printf("Broken pipe.\n");
            exit(8);
        }
        if (ret) {
            sockread = read(clientfd, &socketcommbuff[socketcommbuff_offset], 1024);
            if ((sockread > 10) ||
                ((strncmp(socketcommbuff, COMMANDER_PASSWORD, 8) != 0) &&
                 (strncmp(socketcommbuff, COMMANDER_PASSWORD"\n", 9) != 0) &&
                 (strncmp(socketcommbuff, COMMANDER_PASSWORD"\r\n", 10) != 0) ) ) {
                    replylen = sprintf(replymsg, "!BADPWD!\r\n");
                    wret = write(clientfd, replymsg, replylen);
                    if (wret == -1) {
                        printf("Cannot send reply to client! Connection lost?\n");
                        perror("write()");
                        exit(6);
                    }

                    shutdown(clientfd, SHUT_RDWR);
                    close(clientfd);
                    usleep(1000000);
                    shutdown(listenfd, SHUT_RDWR);
                    close(listenfd);
                    exit(0);
            }
        } else { // timeout
            replylen = sprintf(replymsg, "Timeout. Goodbye!\r\n");
            wret = write(clientfd, replymsg, replylen);
            if (wret == -1) {
                printf("Cannot send reply to client! Connection lost?\n");
                perror("write()");
                exit(6);
            }
            shutdown(clientfd, SHUT_RDWR);
            shutdown(listenfd, SHUT_RDWR);
            close(clientfd);
            close(listenfd);
            exit(0);
        }

        replylen = sprintf(replymsg, "NLAB-MecanumCommander v" COMMANDER_VERSION " - Rover type: 0x%02x firmware: 0x%02x. Ready.\r\n", rover.sysname, rover.firmrev);
        wret = write(clientfd, replymsg, replylen);
        if (wret == -1) {
            printf("Cannot send reply to client! Connection lost?\n");
            perror("write()");
            exit(6);
        }
        sockread = 0;
    }

    // it seems like this doesn't really do anything on this robot... but anyway
    if (dummymode == 0) {
        printf("Enabling motors on main controller.\n");
        rover_enable_motors_main(answer);
        if (rover.config->has_second_controller == 1) {
            printf("Enabling motors on front controller.\n");
            rover_enable_motors_front(answer);
        }
    }

    // ncurses init
    setlocale(LC_CTYPE, "");
    initscr();
    noecho();
    cbreak();
    timeout(500);
    start_color();
    init_pair(1, 15,  0); // feher + fekete
    init_pair(2 , 9,  7); // piros + szurke
    init_pair(3, 10,  0); // zold + fekete
    init_pair(4, 10, 15); // zold + feher
    init_pair(5,  9,  0); // piros + fekete
    init_pair(6, 12,  0); // kek + fekete
    init_pair(7, 11,  0); // sarga + fekete
    init_pair(8, 12,  7); // kek + szurke
    init_pair(9,  7,  0); // szurke + fekete
    init_pair(10, 0,  7); // fekete + szurke
    curs_set(0);

    attron(COLOR_PAIR(1));
    mvprintw(0, 1, "Rover type 0x%x (%s) Firmware revision: 0x%x", rover.sysname, rover.fullname, rover.firmrev);
    attroff(COLOR_PAIR(1));
    attron(COLOR_PAIR(9));
    mvprintw(roverdrawy    , roverdrawx + 5, "☵▇█▇☵");
    mvprintw(roverdrawy + 1, roverdrawx + 5, "▐███▌");
    mvprintw(roverdrawy + 2, roverdrawx + 5, "☵");
    attroff(COLOR_PAIR(9));
    attron(COLOR_PAIR(10));
    mvprintw(roverdrawy + 2, roverdrawx + 6, "▁▁▁");
    attroff(COLOR_PAIR(10));
    attron(COLOR_PAIR(2));
    mvprintw(roverdrawy + 2, roverdrawx + 7, "⬤");
    attroff(COLOR_PAIR(2));
    attron(COLOR_PAIR(9));
    mvprintw(roverdrawy + 2, roverdrawx + 9, "☵");
    attroff(COLOR_PAIR(9));

    attron(A_UNDERLINE | A_BOLD);
    mvprintw(commanddrawy, commanddrawx, "Command               ");
    mvprintw(statusdrawy,  statusdrawx , "Rover status                         ");
    mvprintw(aboutdrawy,   aboutdrawx  , "About                 ");
    attroff(A_UNDERLINE | A_BOLD);

    attron(COLOR_PAIR(1));
    mvprintw(aboutdrawy + 1, aboutdrawx, "NLAB-MecanumCommander");
    mvprintw(aboutdrawy + 2, aboutdrawx + 7, "for Linux v" COMMANDER_VERSION);
    mvprintw(aboutdrawy + 4, aboutdrawx, "See source for more info.");

    mvprintw(commanddrawy + 1, commanddrawx, "X Speed:      0 mm/s");
    mvprintw(commanddrawy + 2, commanddrawx, "Y Speed:      0 mm/s");
    mvprintw(commanddrawy + 3, commanddrawx, "Rotation:     0 mrad/s");

    mvprintw(statusdrawy + 1,  statusdrawx, "Uptime              :  % 6.2lf sec", rover.uptime_sec);
    mvprintw(statusdrawy + 2,  statusdrawx, "Battery             :     %2.2lf V", rover.battery_voltage);
    mvprintw(statusdrawy + 3,  statusdrawx, "Motors          🏍️   :");
    mvprintw(statusdrawy + 4,  statusdrawx + 1, "Speed        M3,M4 : ");
    mvprintw(statusdrawy + 5,  statusdrawx + 1, "Speed        M1,M2 : ");
    mvprintw(statusdrawy + 6,  statusdrawx + 1, "Position     M3,M4 : ");
    mvprintw(statusdrawy + 7,  statusdrawx + 1, "Position     M1,M2 : ");
    mvprintw(statusdrawy + 8,  statusdrawx + 1, "Encoder      M3,M4 : ");
    mvprintw(statusdrawy + 9,  statusdrawx + 1, "Encoder      M1,M2 : ");
    mvprintw(statusdrawy + 10, statusdrawx + 1, "OutputOffset M3,M4 : ");
    mvprintw(statusdrawy + 11, statusdrawx + 1, "OutputOffset M1,M2 : ");
    mvprintw(statusdrawy + 12, statusdrawx + 1, "MotOutCalc   M3,M4 :               %%");
    mvprintw(statusdrawy + 13, statusdrawx + 1, "MotOutCalc   M1,M2 :               %%");
    mvprintw(statusdrawy + 14, statusdrawx + 1, "MeasuredCurr M3,M4 :               A");
    mvprintw(statusdrawy + 15, statusdrawx + 1, "MeasuredCurr M1,M2 :               A");
    mvprintw(statusdrawy + 16, statusdrawx + 1, "MaxCurrent   M3,M4 :               A");
    mvprintw(statusdrawy + 17, statusdrawx + 1, "MaxCurrent   M1,M2 :               A");
    mvprintw(statusdrawy + 18, statusdrawx + 1, "CurrentLimit M3,M4 :               A");
    mvprintw(statusdrawy + 19, statusdrawx + 1, "CurrentLimit M1,M2 :               A");
    attroff(COLOR_PAIR(1));

    attron(COLOR_PAIR(7));
    mvprintw(statusdrawy + 2, statusdrawx + 16, "⚡");
    attroff(COLOR_PAIR(7));


// the fun begins here

    time_last_memmapread = 0;
    time_last_cmdsent = 0;
    time_last_kcmdsent = 0;
    time_last_remotecmd_recv = 0;

    while ((quit == 0) && (got_sigpipe == 0)) {

        int setret = 0;

        ftime(&timestruct);
        time_current = timestruct.time + (timestruct.millitm / 1000.0);

        if (dummymode == 0) {

            if ((time_current - time_last_memmapread) > REPEAT_TIME_SEC_MEMMAPREAD) {

                int fd;

                // heart on
                attron(COLOR_PAIR(5) | A_BOLD);
                mvprintw(statusdrawy + 1, statusdrawx + 16, "♥");
                attroff(COLOR_PAIR(5) | A_BOLD);
                refresh();

                // when using memmapupdate_via_wifi.sh
                if (readmemmapfromfile == 1) {
                    fd = open("memmap_0x10.dat", O_RDONLY);
                    if (fd != -1) {
                        ret = read(fd, rover.memmap_main, 512);
                    }
                    close(fd);
                    fd = open("memmap_0x1F.dat", O_RDONLY);
                    if (fd != -1) {
                        ret = read(fd, rover.memmap_front, 512);
                    }
                    close(fd);
                } else { // read memmap from rover
                    ret = rover_read_full_memmap(rover.memmap_main, ROVER_CONTROLLER_ADDR_REAR, &rover);
                    if (ret == -2) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            stoprobot(usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        errormsg("Fatal error, while reading main memmap! Press a key to quit!", 5);
                        quit = 2;
                        break;
                    }
                    if (ret != 384) {
                        unsigned char errmsg[256];
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            stoprobot(usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        sprintf(errmsg, "Failed to read main memmap correctly (invalid length: %d). Press a key to quit!", ret);
                        errormsg(errmsg, 1);
                        quit = 3;
                        break;
                    }
                    if (rover.config->has_second_controller == 1) {
                        ret = rover_read_full_memmap(rover.memmap_front, ROVER_CONTROLLER_ADDR_FRONT, &rover);
                        if (ret == -2) {
                            if (dummymode == 0) {
                                commandsend_lamp_on();
                                stoprobot(usekcommands, answer);
                                commandsend_lamp_off();
                            }
                            errormsg("Fatal error, while reading front memmap! Press a key to quit!", 5);
                            quit = 2;
                            break;
                        }
                        if (ret != 384) {
                            unsigned char errmsg[256];
                            if (dummymode == 0) {
                                commandsend_lamp_on();
                                stoprobot(usekcommands, answer);
                                commandsend_lamp_off();
                            }
                            sprintf(errmsg, "Failed to read front memmap correctly (invalid length: %d). Press a key to quit!", ret);
                            errormsg(errmsg, 1);
                            quit = 3;
                            break;
                        }
                    }
                }

                time_last_memmapread = timestruct.time + (timestruct.millitm / 1000.0);

            }

        }

        if ((rover.rs485_err_0x10 > 0) || (rover.rs485_err_0x1F > 0)) {
            int alertcolor = 7;
            if ((rover.rs485_err_0x10 >= 5) || (rover.rs485_err_0x1F >= 5)) {
                alertcolor = 5;
            }
            attron(COLOR_PAIR(alertcolor));
            mvprintw(1, 30, "⚠ RS485 CommErr! Main:%03d Front:%03d ⚠", rover.rs485_err_0x10, rover.rs485_err_0x1F);
            attroff(COLOR_PAIR(alertcolor));
        }

        rover.uptime_sec = rover_get_uptime(rover.memmap_main);
        rover.battery_voltage = rover_get_battery_voltage(rover.memmap_main);
        rover.main_motor_status = rover_get_motor_status(rover.memmap_main);
        rover.front_motor_status = rover_get_motor_status(rover.memmap_front);
        rover.motor_m1_status = rover.main_motor_status&1;
        rover.motor_m2_status = rover.main_motor_status>>1;
        rover.motor_m3_status = rover.front_motor_status&1;
        rover.motor_m4_status = rover.front_motor_status>>1;
        rover.front_speed0 = rover_get_speed0(rover.memmap_front);
        rover.front_speed1 = rover_get_speed1(rover.memmap_front);
        rover.main_speed0 = rover_get_speed0(rover.memmap_main);
        rover.main_speed1 = rover_get_speed1(rover.memmap_main);
        rover.front_measured_position0 = rover_get_measured_position0(rover.memmap_front);
        rover.front_measured_position1 = rover_get_measured_position1(rover.memmap_front);
        rover.main_measured_position0 = rover_get_measured_position0(rover.memmap_main);
        rover.main_measured_position1 = rover_get_measured_position1(rover.memmap_main);
        rover.front_encoder_value0 = rover_get_encoder_value0(rover.memmap_front);
        rover.front_encoder_value1 = rover_get_encoder_value1(rover.memmap_front);
        rover.main_encoder_value0 = rover_get_encoder_value0(rover.memmap_main);
        rover.main_encoder_value1 = rover_get_encoder_value1(rover.memmap_main);
        rover.front_outputoffset0 = rover_get_outputoffset0(rover.memmap_front);
        rover.front_outputoffset1 = rover_get_outputoffset1(rover.memmap_front);
        rover.main_outputoffset0 = rover_get_outputoffset0(rover.memmap_main);
        rover.main_outputoffset1 = rover_get_outputoffset1(rover.memmap_main);
        rover.front_motoroutput_calc0 = rover_get_motoroutput_calc0(rover.memmap_front);
        rover.front_motoroutput_calc1 = rover_get_motoroutput_calc1(rover.memmap_front);
        rover.main_motoroutput_calc0 = rover_get_motoroutput_calc0(rover.memmap_main);
        rover.main_motoroutput_calc1 = rover_get_motoroutput_calc1(rover.memmap_main);
        rover.front_measured_current_value0 = rover_get_measured_current_value0(rover.memmap_front);
        rover.front_measured_current_value1 = rover_get_measured_current_value1(rover.memmap_front);
        rover.main_measured_current_value0 = rover_get_measured_current_value0(rover.memmap_main);
        rover.main_measured_current_value1 = rover_get_measured_current_value1(rover.memmap_main);
        rover.front_max_current0 = rover_get_max_current0(rover.memmap_front);
        rover.front_max_current1 = rover_get_max_current1(rover.memmap_front);
        rover.main_max_current0 = rover_get_max_current0(rover.memmap_main);
        rover.main_max_current1 = rover_get_max_current1(rover.memmap_main);
        rover.front_current_limit0 = rover_get_current_limit0(rover.memmap_front);
        rover.front_current_limit1 = rover_get_current_limit1(rover.memmap_front);
        rover.main_current_limit0 = rover_get_current_limit0(rover.memmap_main);
        rover.main_current_limit1 = rover_get_current_limit1(rover.memmap_main);

        attron(COLOR_PAIR(1));
        mvprintw(statusdrawy + 1, statusdrawx + 22, " % 6.2lf sec", rover.uptime_sec);
        mvprintw(statusdrawy + 2, statusdrawx + 22, "    %2.2lf V", rover.battery_voltage);
        attroff(COLOR_PAIR(1));

        if (rover.motor_m1_status == 1) {
            attron(COLOR_PAIR(3));
            mvprintw(statusdrawy + 3, statusdrawx + 22, "On ");
            mvprintw(roverdrawy + 3, roverdrawx + 2, "M1");
            attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(5));
            mvprintw(statusdrawy + 3, statusdrawx + 22, "Off");
            mvprintw(roverdrawy + 3, roverdrawx + 2, "M1");
            attroff(COLOR_PAIR(5));
        }
        if (rover.motor_m2_status == 1) {
            attron(COLOR_PAIR(3));
            mvprintw(statusdrawy + 3, statusdrawx + 26, "On ");
            mvprintw(roverdrawy + 3, roverdrawx + 11, "M2");
            attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(5));
            mvprintw(statusdrawy + 3, statusdrawx + 26, "Off");
            mvprintw(roverdrawy + 3, roverdrawx + 11, "M2");
            attroff(COLOR_PAIR(5));
        }
        if (rover.motor_m3_status == 1) {
            attron(COLOR_PAIR(3));
            mvprintw(statusdrawy + 3, statusdrawx + 30, "On ");
            mvprintw(roverdrawy - 1, roverdrawx + 2, "M3");
            attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(5));
            mvprintw(statusdrawy + 3, statusdrawx + 30, "Off");
            mvprintw(roverdrawy - 1, roverdrawx + 2, "M3");
            attroff(COLOR_PAIR(5));
        }
        if (rover.motor_m4_status == 1) {
            attron(COLOR_PAIR(3));
            mvprintw(statusdrawy + 3, statusdrawx + 34, "On ");
            mvprintw(roverdrawy - 1, roverdrawx + 11, "M4");
            attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(5));
            mvprintw(statusdrawy + 3, statusdrawx + 34, "Off");
            mvprintw(roverdrawy - 1, roverdrawx + 11, "M4");
            attroff(COLOR_PAIR(5));
        }

        attron(COLOR_PAIR(1));
        mvprintw(statusdrawy + 4,  statusdrawx + 22, "% 6d,% 6d", rover.front_speed0, rover.front_speed1);
        mvprintw(statusdrawy + 5,  statusdrawx + 22, "% 6d,% 6d", rover.main_speed0,  rover.main_speed1);
        mvprintw(statusdrawy + 6,  statusdrawx + 22, "% 6d,% 6d", rover.front_measured_position0, rover.front_measured_position1);
        mvprintw(statusdrawy + 7,  statusdrawx + 22, "% 6d,% 6d", rover.main_measured_position0,  rover.main_measured_position1);
        mvprintw(statusdrawy + 8,  statusdrawx + 22, "% 6d,% 6d", rover.front_encoder_value0, rover.front_encoder_value1);
        mvprintw(statusdrawy + 9,  statusdrawx + 22, "% 6d,% 6d", rover.main_encoder_value0,  rover.main_encoder_value1);
        mvprintw(statusdrawy + 10, statusdrawx + 22, "% 6d,% 6d", rover.front_outputoffset0, rover.front_outputoffset1);
        mvprintw(statusdrawy + 11, statusdrawx + 22, "% 6d,% 6d", rover.main_outputoffset0,  rover.main_outputoffset1);
        mvprintw(statusdrawy + 12, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.front_motoroutput_calc0, rover.front_motoroutput_calc1);
        mvprintw(statusdrawy + 13, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.main_motoroutput_calc0,  rover.main_motoroutput_calc1);
        mvprintw(statusdrawy + 14, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.front_measured_current_value0, rover.front_measured_current_value1);
        mvprintw(statusdrawy + 15, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.main_measured_current_value0,  rover.main_measured_current_value1);
        mvprintw(statusdrawy + 16, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.front_max_current0, rover.front_max_current1);
        mvprintw(statusdrawy + 17, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.main_max_current0,  rover.main_max_current1);
        mvprintw(statusdrawy + 18, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.front_current_limit0, rover.front_current_limit1);
        mvprintw(statusdrawy + 19, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover.main_current_limit0,  rover.main_current_limit1);
        attroff(COLOR_PAIR(1));

        // heart off
        if (dummymode == 0) {
            attron(COLOR_PAIR(1));
            mvprintw(statusdrawy + 1, statusdrawx + 16, " ");
            attroff(COLOR_PAIR(1));
        }

        refresh();

        c = -1;

        FD_ZERO(&commfdset);
        FD_SET(0, &commfdset);
        if (remotecontrol == 1) {
            FD_SET(clientfd, &commfdset);
        }

        tv.tv_sec  = 0;
        tv.tv_usec = 100000;

        if (remotecontrol == 1) {
            ret = select(clientfd + 1, &commfdset, NULL, NULL, &tv);
        } else {
            ret = select(1, &commfdset, NULL, NULL, &tv);
        }
        if (ret == -1) {
            endwin();
            perror("select()");
            quit = 1;
            break;
        } else {
            if (got_sigpipe == 1) { break; }
            if (ret) {
                if (FD_ISSET(0, &commfdset)) {
                    c = getch();
                }
                // don't read any new data, while the previous buffer has not been emptied yet
                if ((remotecontrol == 1) && (sockread == 0)) {
                    if (FD_ISSET(clientfd, &commfdset)) {
                        sockread = read(clientfd, &socketcommbuff[socketcommbuff_offset], 1024);
                        socketcommbuff[sockread] = 0;
                        if (sockread == -1) {
                            if (dummymode == 0) {
                                commandsend_lamp_on();
                                stoprobot(usekcommands, answer);
                                commandsend_lamp_off();
                            }
                            errormsg("Socket read error! Press a key to quit!", 1);
                            quit = 4;
                            break;
                        }
                    }
                }
            } else {
                sockread = 0;
            }
        }

        while (sockread > 0) {
                unsigned int sockcommi, copyi, commlen, wret, difflen;
                unsigned char replymsg[16];

                for (sockcommi = 0; ((socketcommbuff[sockcommi] != '\n') && (socketcommbuff[sockcommi] != '\r') && (sockcommi < 1024) && (sockcommi < sockread)); sockcommi++) {}
                if (sockcommi == 1024) {
                    if (dummymode == 0) {
                        commandsend_lamp_on();
                        stoprobot(usekcommands, answer);
                        commandsend_lamp_on();
                    }
                    errormsg("Bad message through socket - Too long (no newline found)! Press a key to quit!", 1);
                    quit = 5;
                    break;
                }

                // first char was a newline
                if (sockcommi == 1) {
                    for (sockcommi = 0; sockcommi < (sockread-1); sockcommi++) {
                        socketcommbuff[sockcommi] = socketcommbuff[sockcommi+1];
                    }
                }
                commlen = sockcommi;

                // if not just an empty line (only /r/n) - commlen/sockcommi == 0
                if ((commlen > 0) && (socketcommbuff[sockcommi-1] == '\r')) {
                    commlen--;
                }
                strncpy(receivedcommand, socketcommbuff, commlen);
                receivedcommand[commlen] = 0;
                difflen = sockread - sockcommi - 1;
                if (difflen != 0) {
                    for (copyi = 0; copyi < difflen; copyi++) {
                        socketcommbuff[copyi] = socketcommbuff[copyi+sockcommi+1];
                    }
                    socketcommbuff[copyi] = 0;
                }
                sockread -= sockcommi + 1;
                socketcommbuff[sockread] = 0;

                if (commlen == 0) { continue; }

                if (commlen != 8) {
                    strncpy(replymsg, "!BADCMD!\r\n", 10);
                    wret = write(clientfd, replymsg, 10);
                    if (wret == -1) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            stoprobot(usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                        quit = 6;
                        break;
                    }
                } else {
                    int badcommand = 1;

                    if (strncmp(receivedcommand, "STOPZERO", 8) == 0) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            stoprobot(usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        rotate = 0;
                        speedX = 0;
                        speedY = 0;
                        badcommand = 0;
                        ftime(&timestruct);
                        time_last_remotecmd_recv = timestruct.time + (timestruct.millitm / 1000.0);
                        strncpy(replymsg, "OKZERO\r\n", 8);
                        wret = write(clientfd, replymsg, 8);
                        if (wret == -1) {
                            if (dummymode == 0) {
                                commandsend_lamp_on();
                                stoprobot(usekcommands, answer);
                                commandsend_lamp_off();
                            }
                            errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                            quit = 6;
                            break;
                        }
                    }

                    if (strncmp(receivedcommand, "ROT", 3) == 0) {
                        int newrot = 0, newrotret;
                        badcommand = 0;
                        newrotret = sscanf(&receivedcommand[3], "%d", &newrot);
                        set_new_rot_value_from_remote = 0;
                        if ((newrotret != 1) || (newrot < -10000) || (newrot > 10000)) {
                            strncpy(replymsg, "!BADROT!\r\n", 10);
                            wret = write(clientfd, replymsg, 10);
                            if (wret == -1) {
                                if (dummymode == 0) {
                                    commandsend_lamp_on();
                                    stoprobot(usekcommands, answer);
                                    commandsend_lamp_off();
                                }
                                errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                                quit = 6;
                                break;
                            }
                        } else {
                            rotate = newrot;
                            set_new_rot_value_from_remote = 1;
                            ftime(&timestruct);
                            time_last_remotecmd_recv = timestruct.time + (timestruct.millitm / 1000.0);
                        }
                    }

                    if (strncmp(receivedcommand, "SPX", 3) == 0) {
                        int newspx = 0, newspxret;
                        badcommand = 0;
                        newspxret = sscanf(&receivedcommand[3], "%d", &newspx);
                        set_new_spx_value_from_remote = 0;
                        if ((newspxret != 1) || (newspx < -10000) || (newspx > 10000)) {
                            strncpy(replymsg, "!BADSPX!\r\n", 10);
                            wret = write(clientfd, replymsg, 10);
                            if (wret == -1) {
                                if (dummymode == 0) {
                                    commandsend_lamp_on();
                                    stoprobot(usekcommands, answer);
                                    commandsend_lamp_off();
                                }
                                errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                                quit = 6;
                                break;
                            }
                        } else {
                            speedX = newspx;
                            set_new_spx_value_from_remote = 1;
                            ftime(&timestruct);
                            time_last_remotecmd_recv = timestruct.time + (timestruct.millitm / 1000.0);
                        }
                    }

                    if (strncmp(receivedcommand, "SPY", 3) == 0) {
                        int newspy=0, newspyret;

                        badcommand = 0;
                        newspyret = sscanf(&receivedcommand[3], "%d", &newspy);
                        set_new_spy_value_from_remote = 0;
                        if ((newspyret != 1) || (newspy < -10000) || (newspy > 10000)) {
                            strncpy(replymsg, "!BADSPY!\r\n", 10);
                            wret = write(clientfd, replymsg, 10);
                            if (wret == -1) {
                                if (dummymode == 0) {
                                    commandsend_lamp_on();
                                    stoprobot(usekcommands, answer);
                                    commandsend_lamp_off();
                                }
                                errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                                quit = 6;
                                break;
                            }
                        } else {
                            speedY = newspy;
                            set_new_spy_value_from_remote = 1;
                            ftime(&timestruct);
                            time_last_remotecmd_recv = timestruct.time + (timestruct.millitm / 1000.0);
                        }
                    }

                if (badcommand == 1) {
                    strncpy(replymsg, "!BADCMD!\r\n", 10);
                    wret = write(clientfd, replymsg, 10);
                    if (wret == -1) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            stoprobot(usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                        quit = 6;
                        break;
                    }
                }

            }
        }

        if (c != -1) {
            // stop
            if ((c == 32) || (c == 53) || (c == 48) || (c == 120) || (c == 10) || (c == 13) || (c == 46) || (c == 126)) {
                speedX = 0;
                speedY = 0;
                rotate = 0;
                if (dummymode == 0) {
                    commandsend_lamp_on();
                    stoprobot(usekcommands, answer);
                    commandsend_lamp_off();
                }
            }

            if (c == 27) {
                timeout(0);
                c2 = getch();
                if (c2 == -1) {
                    quit = 1;
                    break;
                }
                if (c2 == 91) {
                    c3 = getch();
                    switch (c3) {
                        case 49:
                            c4 = getch();
                            c5 = getch();
                            c6 = getch();
                            if ((c4 == 59) && (c5 == 50)) {
                                switch(c6) {
                                    case 65: c = 87; break;
                                    case 66: c = 83; break;
                                    case 67: c = 68; break;
                                    case 68: c = 65; break;
                                    case 72: c = 81; break;
                                }
                            }
                            break;

                        case 53:
                            c4 = getch();
                            if (c4 == 59) {
                                c5 = getch();
                                c6 = getch();
                                if ((c5 == 50) && (c6 == 126)) { c = 69; }
                            } else {
                                if (c4 == 126) { c = 101; }
                            }
                            break;

                        case 65: c = 119; break;
                        case 66: c = 115; break;
                        case 67: c = 100; break;
                        case 68: c =  97; break;
                        case 69: c =  32; break;
                        case 72: c = 113; break;
                    }
                }

                timeout(500);
            }

            switch (c) {
                case 119:
                case  56: speedX +=  25; break;
                case 115:
                case  50: speedX -=  25; break;
                case  97:
                case  52: speedY +=  25; break;
                case 100:
                case  54: speedY -=  25; break;
                case 113:
                case  55: rotate += 250; break;
                case 101:
                case  57: rotate -= 250; break;
                case  87: speedX += 100; break;
                case  83: speedX -= 100; break;
                case  65: speedY += 100; break;
                case  68: speedY -= 100; break;
                case  81: rotate += 500; break;
                case  69: rotate -= 500; break;
            }

        } else { // c == -1 - no key was pressed
            if (keyboardmode == 2) {
                speedX = 0;
                speedY = 0;
                rotate = 0;
            }
        }

        if (speedX >  LIMIT_SPEED_X)   { speedX =  LIMIT_SPEED_X; }
        if (speedX < -LIMIT_SPEED_X)   { speedX = -LIMIT_SPEED_X; }
        if (speedY >  LIMIT_SPEED_Y)   { speedY =  LIMIT_SPEED_Y; }
        if (speedY < -LIMIT_SPEED_Y)   { speedY = -LIMIT_SPEED_Y; }
        if (rotate >  LIMIT_SPEED_ROT) { rotate =  LIMIT_SPEED_ROT; }
        if (rotate < -LIMIT_SPEED_ROT) { rotate = -LIMIT_SPEED_ROT; }

        attron(COLOR_PAIR(1));
        mvprintw(commanddrawy + 1, commanddrawx + 10, "% 5d", speedX);
        mvprintw(commanddrawy + 2, commanddrawx + 10, "% 5d", speedY);
        mvprintw(commanddrawy + 3, commanddrawx + 10, "% 5d", rotate);
        attroff(COLOR_PAIR(1));
        if (speedX > 0) {
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(commanddrawy + 1, commanddrawx + 23, "↑");
            mvprintw(roverdrawy - 2, roverdrawx + 6, "↑↑↑");
            attroff(COLOR_PAIR(6) | A_BOLD);
            mvprintw(roverdrawy + 4, roverdrawx + 6, "   ");
        } else {
            if (speedX < 0) {
                attron(COLOR_PAIR(6) | A_BOLD);
                mvprintw(commanddrawy + 1, commanddrawx + 23, "↓");
                mvprintw(roverdrawy + 4, roverdrawx + 6, "↓↓↓");
                attroff(COLOR_PAIR(6) | A_BOLD);
                mvprintw(roverdrawy - 2, roverdrawx + 6, "   ");
            } else {
                mvprintw(commanddrawy + 1, commanddrawx + 23, " ");
                mvprintw(roverdrawy - 2, roverdrawx + 6, "   ");
                mvprintw(roverdrawy + 4, roverdrawx + 6, "   ");
            }
        }

        if (speedY < 0) {
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(commanddrawy + 2, commanddrawx + 23, "→");
            mvprintw(roverdrawy + 1, roverdrawx + 12, "→→→");
            attroff(COLOR_PAIR(6) | A_BOLD);
            mvprintw(roverdrawy + 1, roverdrawx, "   ");
        } else {
            if (speedY > 0) {
                attron(COLOR_PAIR(6) | A_BOLD);
                mvprintw(commanddrawy + 2, commanddrawx + 23, "←");
                mvprintw(roverdrawy + 1, roverdrawx, "←←←");
                attroff(COLOR_PAIR(6) | A_BOLD);
                mvprintw(roverdrawy + 1, roverdrawx + 12, "   ");
            } else {
                mvprintw(commanddrawy + 2, commanddrawx + 23, " ");
                mvprintw(roverdrawy + 1, roverdrawx, "   ");
                mvprintw(roverdrawy + 1, roverdrawx + 12, "   ");
            }
        }

        if (rotate < 0) {
            attron(COLOR_PAIR(6) | A_BOLD);
            mvprintw(commanddrawy + 3, commanddrawx + 23, "↻");
            attroff(COLOR_PAIR(6) | A_BOLD);
            attron(COLOR_PAIR(8) | A_BOLD);
            mvprintw(roverdrawy + 1, roverdrawx + 7, "↻");
            attroff(COLOR_PAIR(8) | A_BOLD);
        } else {
            if (rotate > 0) {
                attron(COLOR_PAIR(6) | A_BOLD);
                mvprintw(commanddrawy + 3, commanddrawx + 23, "↺");
                attroff(COLOR_PAIR(6) | A_BOLD);
                attron(COLOR_PAIR(8) | A_BOLD);
                mvprintw(roverdrawy + 1, roverdrawx + 7, "↺");
                attroff(COLOR_PAIR(8) | A_BOLD);
            } else {
                mvprintw(commanddrawy + 3, commanddrawx + 23, " ");
                attron(COLOR_PAIR(9));
                mvprintw(roverdrawy + 1, roverdrawx + 7, "█");
                attroff(COLOR_PAIR(9));
            }
        }

        ftime(&timestruct);
        time_current = timestruct.time + (timestruct.millitm / 1000.0);

        if (remotecontrol == 1) {
            if ((time_current - time_last_remotecmd_recv) > REPEAT_TIME_SEC_REMOTECMDRECV) {
                if (remotecmd_timed_out == 0) {
                    unsigned char replymsg[16];
                    int wret;

                    if (dummymode == 0) {
                        commandsend_lamp_on();
                        stoprobot(usekcommands, answer);
                        commandsend_lamp_off();
                    }
                    speedX = 0;
                    speedY = 0;
                    rotate = 0;
                    set_new_spx_value_from_remote = 0;
                    set_new_spy_value_from_remote = 0;
                    set_new_rot_value_from_remote = 0;
                    strncpy(replymsg, "!NOCMST!\r\n", 10);
                    wret = write(clientfd, replymsg, 10);
                    remotecmd_timed_out = 1;
                    if (wret == -1) {
                        errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                        perror("write()");
                        quit = 6;
                        break;
                    }
                }
            } else {
                remotecmd_timed_out = 0;
            }
        }

        if ((usekcommands == 1) && ((time_current - time_last_kcmdsent) > REPEAT_TIME_SEC_KCMDSENT)) {
            if (dummymode == 0) {
                commandsend_lamp_on();
                setret = rover_kset_XYrotation_speed(speedX, speedY, rotate, answer);
                commandsend_lamp_off();
                usleep(100);
                time_last_kcmdsent = time_current;
            }
        }

        if ((usekcommands == 0) && ((time_current - time_last_cmdsent) > REPEAT_TIME_SEC_CMDSENT)) {

            if ( (repeatcommands == 1) || (set_new_spx_value_from_remote == 1) || (prevspeedX != speedX) ) {
// pozitiv elore -- negativ hatra
                if (dummymode == 0) {
                    commandsend_lamp_on();
                    setret=rover_set_X_speed(speedX, answer);
                    commandsend_lamp_off();
                    usleep(100);
                    time_last_cmdsent = time_current;
                }
                prevspeedX = speedX;
            }

            if ( (repeatcommands == 1) || (set_new_spy_value_from_remote == 1) || (prevspeedY != speedY) ) {
// pozitiv balra - negativ jobbra
                if (dummymode == 0) {
                    commandsend_lamp_on();
                    setret = rover_set_Y_speed(speedY, answer);
                    commandsend_lamp_off();
                    usleep(100);
                    time_last_cmdsent = time_current;
                }
                prevspeedY = speedY;
            }

            if ( (repeatcommands == 1) || (set_new_rot_value_from_remote == 1) || (prevrotate != rotate) ) {
// pozitiv balra forgas (100 is meg eleg lassu)
                if (dummymode == 0) {
                    commandsend_lamp_on();
                    setret = rover_set_rotation_speed(rotate, answer);
                    commandsend_lamp_off();
                    usleep(100);
                    time_last_cmdsent = time_current;
                }
                prevrotate = rotate;
            }

            if ( (set_new_spx_value_from_remote == 1) || (set_new_spy_value_from_remote == 1) || (set_new_rot_value_from_remote == 1) ) {

                unsigned char replymsg[16];
                int wret;

                if (set_new_spx_value_from_remote == 1) {
                    set_new_spx_value_from_remote = 0;
                    if (setret == 0) {
                        strncpy(replymsg, "OKSPX\r\n", 7);
                        wret = write(clientfd, replymsg, 7);
                    } else {
                        strncpy(replymsg, "!ERRSPX!\r\n", 10);
                        wret = write(clientfd, replymsg, 10);
                    }
                }

                if (set_new_spy_value_from_remote == 1) {
                    set_new_spy_value_from_remote = 0;
                    if (setret == 0) {
                        strncpy(replymsg, "OKSPY\r\n", 7);
                        wret = write(clientfd, replymsg, 7);
                    } else {
                        strncpy(replymsg, "!ERRSPY!\r\n", 10);
                        wret = write(clientfd, replymsg, 10);
                    }
                }

                if (set_new_rot_value_from_remote == 1) {
                    set_new_rot_value_from_remote = 0;
                    if (setret == 0) {
                        strncpy(replymsg, "OKROT\r\n", 7);
                        wret = write(clientfd, replymsg, 7);
                    } else {
                        strncpy(replymsg, "!ERRROT!\r\n", 10);
                        wret = write(clientfd, replymsg, 10);
                    }
                }

                if (wret == -1) {
                    if (dummymode == 0) {
                        commandsend_lamp_on();
                        stoprobot(usekcommands, answer);
                        commandsend_lamp_off();
                    }
                    errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                    quit = 6;
                    break;
                }

            }

        }

    }

    // ncurses close
    endwin();

    if (remotecontrol == 1) {
        unsigned char replymsg[24];
        int wret;

        strncpy(replymsg, "Closing. Byebye!\r\n", 18);
        wret = write(clientfd, replymsg, 17);
        if (wret == -1) {
            if (dummymode == 0) {
                stoprobot(usekcommands, answer);
            }
            printf("Could not say goodbye to client... Connection lost?");
            perror("write()");
            quit = 6;
        }

        close(clientfd);
        close(listenfd);
    }

    switch (quit) {
        case 2: printf("Fatal error happened while reading memmap!\n"); break;
        case 3: printf("Failed to read memmap correctly (invalid length)!\n"); break;
        case 4: printf("Error while reading from socket()!\n"); break;
        case 5: printf("Bad message received through socket - Too long (no newline found)!\n"); break;
        case 6: printf("Could not reply to client! Connection was lost maybe?\n"); break;
    }

    if (got_sigpipe == 1) { printf("Got SIGPIPE! Connection to client was lost maybe?\n"); }

    if (dummymode == 0) {
        printf("Setting X+Y+Rot speed to zero.\n");
        stoprobot(usekcommands, answer);
        printf("Disabling motors on main controller.\n");
        rover_disable_motors_main(answer);
        printf("Disabling motors on front controller.\n");
        rover_disable_motors_front(answer);
    }

    return 0;

}
