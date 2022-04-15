/*
    NLAB-MecanumCommander for Linux, a simple bridge to control VStone MecanumRover 2.1 / VStone MegaRover 3
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021
    version 0.50
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
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include "mecanumrover_commlib.h"

#define COMMANDER_VERSION  "0.50"
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


void stoprobot(struct roverstruct *rover, char usekcommands, char *answer) {
    if (usekcommands == 1) {
        rover_kset_STOP(answer);
    } else {
        rover_set_XYrotation_speed_to_zero(rover, answer);
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


void read_memmap_files(struct roverstruct *rover) {

    int fd;

    fd = open("memmap_0x10.dat", O_RDONLY);
    if (fd != -1) {
        read(fd, rover->memmap_main, 512);
    }
    close(fd);
    fd = open("memmap_0x1F.dat", O_RDONLY);
    if (fd != -1) {
        read(fd, rover->memmap_second, 512);
    }
    close(fd);

}


void logmsg(int logfd, double time_start, const char *msg) {
    double time_curr;
    char logmsg[1024];
    struct timeval timestruct;

    gettimeofday(&timestruct, NULL);
    time_curr = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;
    sprintf(logmsg, "%9.3f %s\n", time_curr - time_start, msg);
    write(logfd, logmsg, strlen(logmsg));
}


int main() {

    int ret;
    unsigned char answer[BUFFER_SIZE];

    struct roverstruct rover;

    struct timeval timestruct;
    double time_start, time_current, time_last_memmapread, time_last_cmdsent, time_last_kcmdsent, time_last_remotecmd_recv;
    unsigned char remotecmd_timed_out=1;

    int logfd;
    char logstring[512];

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
    unsigned int udp_packetno=0, udp_lastpacketno=0;
    unsigned char socketcommbuff[BUFFER_SIZE+1];
    unsigned char receivedcommand[16];
    unsigned char set_new_spx_value_from_remote=0;
    unsigned char set_new_spy_value_from_remote=0;
    unsigned char set_new_rot_value_from_remote=0;

    unsigned char main_motor_status, second_motor_status;

    unsigned char remotecontrol=0;      // set to 1 to listen on tcp/3475 for easy remote control
    unsigned char dummymode=0;          // for testing, do not send real commands to the robot
    unsigned char keyboardmode=1;       // for future use...
    unsigned char repeatcommands=1;     // repeat commands every REPEAT_TIME_SEC_CMDSENT, so "commandtimeout" on the robot's controller won't trigger
    unsigned char usekcommands=0;       // use the "triple command set"
    unsigned char readmemmapfromfile=0; // do not get the memmap from the robot, instead read it from a file
    unsigned char remotecontrolproto=0; // 0 - TCP, 1 - UDP


// create logfile
    logfd = open("mecanumcommander.log", O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP);
    if (logfd == -1) {
        printf("Cannot create logfile: mecanumcommander.log\n");
        exit(1);
    }

    gettimeofday(&timestruct, NULL);
    time_start = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;

    logmsg(logfd, time_start, "Initializing");

    if (dummymode == 1) {
        int fd;

        fd = open("memmap_sample.dat", O_RDONLY);
        if (fd == -1) {
            perror("open(memmap_sample.dat)");
            exit(1);
        }
        if (read(fd, rover.memmap_main, 386) == -1) {
            perror("read() memmap first part");
            exit(1);
        }
        if (read(fd, rover.memmap_second, 386) == -1) {
            perror("read() memmap second part");
            exit(1);
        }
        close(fd);

        if (rover_identify_from_main_memmap(&rover) == 1 ) {
            printf("Unknown rover type: 0x%X!\n", rover.sysname);
            exit(1);
        }

    } else {

        if (check_serial_dev() == -1) {
            printf("Cannot open serial port: %s!\n", DEVFILE);
            exit(1);
        }

        if (readmemmapfromfile == 1) {

            if (access("memmap_0x10.dat", R_OK) != 0) {
                printf("Cannot access memmap_0x10.dat!\n");
                exit(1);
            }
            read_memmap_files(&rover);  // when using memmapupdate_via_wifi.sh

            if (rover_identify_from_main_memmap(&rover) == 1 ) {
                printf("Unknown rover type: 0x%X!\n", rover.sysname);
                exit(1);
            }

        } else {

            if (rover_identify(&rover) == 0) {
                printf("Rover found: 0x%x:%s FWRev: 0x%x\n", rover.sysname, rover.fullname, rover.firmrev);
            } else {
                printf("Unknown rover type: 0x%X!\n", rover.sysname);
                exit(1);
            }

        }

    }

    sprintf(logstring, "Rover type: 0x%x:%s FWRev: 0x%x", rover.sysname, rover.fullname, rover.firmrev);
    logmsg(logfd, time_start, logstring);

    // bind to tcp/3475 or udp/3475
    if (remotecontrol == 1) {
        typedef void (*sighandler_t)(int);
        sighandler_t sigret;
        unsigned char replymsg[128];
        int replylen, wret;
        int enable=1;

        bzero(&serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        serv_addr.sin_port = htons(3475);

        if (remotecontrolproto == 0 ) { // TCP
            listenfd = socket(AF_INET, SOCK_STREAM, 0);
            if (listenfd == -1) {
                perror("socket(TCP)");
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
            logmsg(logfd, time_start, "Waiting for connection on tcp/3475");
            clientfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
            if (clientfd == -1) {
                perror("accept()");
                exit(3);
            }
            replylen = sprintf(replymsg, "I'm NLAB-MecanumCommander. Please authenticate yourself.\r\n");
            wret = write(clientfd, replymsg, replylen);
            logmsg(logfd, time_start, "Welcome string sent to client");
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
                sockread = read(clientfd, &socketcommbuff[socketcommbuff_offset], BUFFER_SIZE);
                if ((sockread > 10) ||
                    ((strncmp(socketcommbuff, COMMANDER_PASSWORD, 8) != 0) &&
                     (strncmp(socketcommbuff, COMMANDER_PASSWORD"\n", 9) != 0) &&
                     (strncmp(socketcommbuff, COMMANDER_PASSWORD"\r\n", 10) != 0) ) ) {
                        replylen = sprintf(replymsg, "!BADPWD!\r\n");
                        wret = write(clientfd, replymsg, replylen);
                        logmsg(logfd, time_start, "Bad password!");
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
                logmsg(logfd, time_start, "Authentication timed out");
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
            logmsg(logfd, time_start, "Welcome message sent to client");
            if (wret == -1) {
                printf("Cannot send reply to client! Connection lost?\n");
                perror("write()");
                exit(6);
            }
            sockread = 0;

        } else { // UDP

            listenfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (listenfd == -1) {
                perror("socket(UDP)");
                exit(3);
            }

            ret = bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
            if (ret == -1) {
                perror("bind(INADDR_ANY/udp/3475)");
                exit(3);
            }

        }
    }

    // it seems like this doesn't really do anything on this robot... but anyway
    if (dummymode == 0) {
        printf("Enabling motors on main controller.\n");
        rover_enable_motors(&rover, rover.regs->controller_addr_main, answer);
        logmsg(logfd, time_start, "Enabled motors on main controller");
        if (rover.config->has_second_controller == 1) {
            printf("Enabling motors on second controller.\n");
            rover_enable_motors(&rover, rover.regs->controller_addr_second, answer);
            logmsg(logfd, time_start, "Enabled motors on second controller");
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

    switch (rover.sysname) {
        case SYSNAME_MECANUMROVER21:
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
            break;

        case SYSNAME_MEGAROVER3:
            attron(COLOR_PAIR(5));
            mvprintw(roverdrawy - 1, roverdrawx + 6, "▂");
            attroff(COLOR_PAIR(5));
            attron(COLOR_PAIR(9));
            mvprintw(roverdrawy    , roverdrawx + 5, "≣███≣");
            mvprintw(roverdrawy + 1, roverdrawx + 7, "█▊");
//            mvprintw(roverdrawy + 1, roverdrawx + 7, "█▉");
            attroff(COLOR_PAIR(9));
            attron(COLOR_PAIR(10));
            mvprintw(roverdrawy + 1, roverdrawx + 6, "▎");
            mvprintw(roverdrawy + 2, roverdrawx + 7, "▇");
            attroff(COLOR_PAIR(10));
            break;

        default:
            attron(COLOR_PAIR(9));
            mvprintw(roverdrawy    , roverdrawx + 5, "☵███☵");
            mvprintw(roverdrawy + 2, roverdrawx + 6, "███");
            attroff(COLOR_PAIR(9));
            attron(COLOR_PAIR(2));
            mvprintw(roverdrawy + 1, roverdrawx + 7, "⬤");
            attroff(COLOR_PAIR(2));
    }

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
    if (rover.config->has_Y_speed == 1) {
        mvprintw(commanddrawy + 2, commanddrawx, "Y Speed:      0 mm/s");
    }
    mvprintw(commanddrawy + 3, commanddrawx, "Rotation:     0 mrad/s");

    mvprintw(statusdrawy + 1,  statusdrawx, "Uptime              :");
    mvprintw(statusdrawy + 2,  statusdrawx, "Battery             :");
    mvprintw(statusdrawy + 3,  statusdrawx, "Motors          🏍️   :");

    switch (rover.sysname) {

        case SYSNAME_MECANUMROVER21:
            mvprintw(statusdrawy +  4, statusdrawx + 1, "Speed        M3,M4 : ");
            mvprintw(statusdrawy +  5, statusdrawx + 1, "Speed        M1,M2 : ");
            mvprintw(statusdrawy +  6, statusdrawx + 1, "Position     M3,M4 : ");
            mvprintw(statusdrawy +  7, statusdrawx + 1, "Position     M1,M2 : ");
            mvprintw(statusdrawy +  8, statusdrawx + 1, "Encoder      M3,M4 : ");
            mvprintw(statusdrawy +  9, statusdrawx + 1, "Encoder      M1,M2 : ");
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
            break;

        case SYSNAME_MEGAROVER3:
            mvprintw(statusdrawy +  5, statusdrawx + 1, "MotorSpeed M1,M2 : ");
            mvprintw(statusdrawy +  9, statusdrawx + 1, "Encoder    M1,M2 : ");
//            mvprintw(statusdrawy + 15, statusdrawx + 1, "MeasuredCurr M1,M2 :               A");
            break;
    }
    attroff(COLOR_PAIR(1));

    attron(COLOR_PAIR(7));
    mvprintw(statusdrawy + 2, statusdrawx + 16, "⚡");
    attroff(COLOR_PAIR(7));


// the fun begins here

    time_last_memmapread = 0;
    time_last_cmdsent = 0;
    time_last_kcmdsent = 0;
    time_last_remotecmd_recv = 0;

    logmsg(logfd, time_start, "Start");

    while ((quit == 0) && (got_sigpipe == 0)) {

        int setret = 0;

        gettimeofday(&timestruct, NULL);
        time_current = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;

        if (dummymode == 0) {

            if ((time_current - time_last_memmapread) > REPEAT_TIME_SEC_MEMMAPREAD) {

                int fd;

                // heart on
                attron(COLOR_PAIR(5) | A_BOLD);
                mvprintw(statusdrawy + 1, statusdrawx + 16, "♥");
                attroff(COLOR_PAIR(5) | A_BOLD);
                refresh();

                if (readmemmapfromfile == 1) {
                    logmsg(logfd, time_start, "Reading memmap from files");
                    read_memmap_files(&rover);  // when using memmapupdate_via_wifi.sh
                } else { // read memmap from rover
                    logmsg(logfd, time_start, "Reading main memmap from robot");
                    ret = rover_read_full_memmap(rover.memmap_main, rover.regs->controller_addr_main, &rover);
                    if (ret == -2) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            logmsg(logfd, time_start, "Stoprobot");
                            stoprobot(&rover, usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        logmsg(logfd, time_start, "Err: Fatal error, while reading main memmap! (2)");
                        errormsg("Fatal error, while reading main memmap! Press a key to quit!", 5);
                        quit = 2;
                        break;
                    }
                    if (ret != 384) {
                        unsigned char errmsg[256];
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            logmsg(logfd, time_start, "Stoprobot");
                            stoprobot(&rover, usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        logmsg(logfd, time_start, "Err: Failed to read main memmap correctly (invalid length) (3)");
                        sprintf(errmsg, "Failed to read main memmap correctly (invalid length: %d). Press a key to quit!", ret);
                        errormsg(errmsg, 1);
                        quit = 3;
                        break;
                    }
                    if (rover.config->has_second_controller == 1) {
                        logmsg(logfd, time_start, "Reading second memmap from robot");
                        ret = rover_read_full_memmap(rover.memmap_second, rover.regs->controller_addr_second, &rover);
                        if (ret == -2) {
                            if (dummymode == 0) {
                                commandsend_lamp_on();
                                logmsg(logfd, time_start, "Stoprobot");
                                stoprobot(&rover, usekcommands, answer);
                                commandsend_lamp_off();
                            }
                            logmsg(logfd, time_start, "Err: Fatal error, while reading second memmap! (2)");
                            errormsg("Fatal error, while reading second memmap! Press a key to quit!", 5);
                            quit = 2;
                            break;
                        }
                        if (ret != 384) {
                            unsigned char errmsg[256];
                            if (dummymode == 0) {
                                commandsend_lamp_on();
                                logmsg(logfd, time_start, "Stoprobot");
                                stoprobot(&rover, usekcommands, answer);
                                commandsend_lamp_off();
                            }
                            logmsg(logfd, time_start, "Err: Failed to read second memmap correctly (invalid length) (3)");
                            sprintf(errmsg, "Failed to read second memmap correctly (invalid length: %d). Press a key to quit!", ret);
                            errormsg(errmsg, 1);
                            quit = 3;
                            break;
                        }
                    }
                }

                gettimeofday(&timestruct, NULL);
                time_last_memmapread = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;

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

        attron(COLOR_PAIR(1));
        mvprintw(statusdrawy + 1, statusdrawx + 22, " % 6.2lf sec", rover_get_uptime(&rover));
        mvprintw(statusdrawy + 2, statusdrawx + 22, "    %2.2lf V", rover_get_battery_voltage(&rover));
        attroff(COLOR_PAIR(1));

        main_motor_status = rover_get_motor_status(&rover, rover.memmap_main);
        if ((main_motor_status & 1) == 1) {
            attron(COLOR_PAIR(3));
            mvprintw(statusdrawy + 3, statusdrawx + 22, "On ");
            if (rover.sysname == SYSNAME_MECANUMROVER21) {
                mvprintw(roverdrawy + 3, roverdrawx + 2, "M1");
            } else {
                mvprintw(roverdrawy - 1, roverdrawx + 2, "M1");
            }
            attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(5));
            mvprintw(statusdrawy + 3, statusdrawx + 22, "Off");
            if (rover.sysname == SYSNAME_MECANUMROVER21) {
                mvprintw(roverdrawy + 3, roverdrawx + 2, "M1");
            } else {
                mvprintw(roverdrawy - 1, roverdrawx + 2, "M1");
            }
            attroff(COLOR_PAIR(5));
        }
        if ((main_motor_status >> 1) == 1) {
            attron(COLOR_PAIR(3));
            mvprintw(statusdrawy + 3, statusdrawx + 26, "On ");
            if (rover.sysname == SYSNAME_MECANUMROVER21) {
                mvprintw(roverdrawy + 3, roverdrawx + 11, "M2");
            } else {
                mvprintw(roverdrawy - 1, roverdrawx + 11, "M2");
            }
            attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(5));
            mvprintw(statusdrawy + 3, statusdrawx + 26, "Off");
            if (rover.sysname == SYSNAME_MECANUMROVER21) {
                mvprintw(roverdrawy + 3, roverdrawx + 11, "M2");
            } else {
                mvprintw(roverdrawy - 1, roverdrawx + 11, "M2");
            }
            attroff(COLOR_PAIR(5));
        }
        if (rover.config->motor_count == 4) {
            second_motor_status = rover_get_motor_status(&rover, rover.memmap_second);

            if ((second_motor_status & 1) == 1) {
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
            if ((second_motor_status >> 1) == 1) {
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
        }

        attron(COLOR_PAIR(1));
        switch (rover.sysname) {

            case SYSNAME_MECANUMROVER21:
                mvprintw(statusdrawy +  4, statusdrawx + 22, "% 6d,% 6d", rover_get_speed0(&rover, rover.memmap_second), rover_get_speed1(&rover, rover.memmap_second));
                mvprintw(statusdrawy +  5, statusdrawx + 22, "% 6d,% 6d", rover_get_speed0(&rover, rover.memmap_main),   rover_get_speed1(&rover, rover.memmap_main));
                mvprintw(statusdrawy +  6, statusdrawx + 22, "% 6d,% 6d", rover_get_measured_position0(&rover, rover.memmap_second), rover_get_measured_position1(&rover, rover.memmap_second));
                mvprintw(statusdrawy +  7, statusdrawx + 22, "% 6d,% 6d", rover_get_measured_position0(&rover, rover.memmap_main),   rover_get_measured_position1(&rover, rover.memmap_main));
                mvprintw(statusdrawy +  8, statusdrawx + 22, "% 6d,% 6d", rover_get_encoder_value0(&rover, rover.memmap_second), rover_get_encoder_value1(&rover, rover.memmap_second));
                mvprintw(statusdrawy +  9, statusdrawx + 22, "% 6d,% 6d", rover_get_encoder_value0(&rover, rover.memmap_main),   rover_get_encoder_value1(&rover, rover.memmap_main));
                mvprintw(statusdrawy + 10, statusdrawx + 22, "% 6d,% 6d", rover_get_outputoffset0(&rover, rover.memmap_second), rover_get_outputoffset1(&rover, rover.memmap_second));
                mvprintw(statusdrawy + 11, statusdrawx + 22, "% 6d,% 6d", rover_get_outputoffset0(&rover, rover.memmap_main),   rover_get_outputoffset1(&rover, rover.memmap_main));
                mvprintw(statusdrawy + 12, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_motoroutput_calc0(&rover, rover.memmap_second), rover_get_motoroutput_calc1(&rover, rover.memmap_second));
                mvprintw(statusdrawy + 13, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_motoroutput_calc0(&rover, rover.memmap_main),   rover_get_motoroutput_calc1(&rover, rover.memmap_main));
                mvprintw(statusdrawy + 14, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_measured_current_value0(&rover, rover.memmap_second), rover_get_measured_current_value1(&rover, rover.memmap_second));
                mvprintw(statusdrawy + 15, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_measured_current_value0(&rover, rover.memmap_main),   rover_get_measured_current_value1(&rover, rover.memmap_main));
                mvprintw(statusdrawy + 16, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_max_current0(&rover, rover.memmap_second), rover_get_max_current1(&rover, rover.memmap_second));
                mvprintw(statusdrawy + 17, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_max_current0(&rover, rover.memmap_main),   rover_get_max_current1(&rover, rover.memmap_main));
                mvprintw(statusdrawy + 18, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_current_limit0(&rover, rover.memmap_second), rover_get_current_limit1(&rover, rover.memmap_second));
                mvprintw(statusdrawy + 19, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_current_limit0(&rover, rover.memmap_main),   rover_get_current_limit1(&rover, rover.memmap_main));
                break;

            case SYSNAME_MEGAROVER3:
                mvprintw(statusdrawy +  5, statusdrawx + 22, "% 8d,% 8d", rover_get_motorspeed0(&rover, rover.memmap_main), rover_get_motorspeed1(&rover, rover.memmap_main));
                mvprintw(statusdrawy +  9, statusdrawx + 22, "% 8d,% 8d", rover_get_encoder_value0(&rover, rover.memmap_main), rover_get_encoder_value1(&rover, rover.memmap_main));
//                mvprintw(statusdrawy + 15, statusdrawx + 22, "% 6.2lf,% 6.2lf", rover_get_measured_current_value0(&rover, rover.memmap_main), rover_get_measured_current_value1(&rover, rover.memmap_main));
                break;
        }
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
            if (remotecontrolproto == 0) { // TCP
                FD_SET(clientfd, &commfdset);
            } else { // UDP
                FD_SET(listenfd, &commfdset);
            }
        }

        tv.tv_sec  = 0;
        tv.tv_usec = 100000;

        if (remotecontrol == 1) {
            if (remotecontrolproto == 0) { // TCP
                ret = select(clientfd + 1, &commfdset, NULL, NULL, &tv);
            } else { // UDP
                ret = select(listenfd + 1, &commfdset, NULL, NULL, &tv);
            }
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
                    logmsg(logfd, time_start, "Keypress:");
                    logmsg(logfd, time_start, &c);
                }
                // don't read any new data, while the previous buffer has not been emptied yet
                if ( (remotecontrol == 1) && (sockread == 0) ) {
                    if (remotecontrolproto == 0) { // TCP
                        if (FD_ISSET(clientfd, &commfdset)) {
                            sockread = read(clientfd, &socketcommbuff[socketcommbuff_offset], BUFFER_SIZE);
                            socketcommbuff[sockread] = 0;
                            if (sockread == 0) {
                                logmsg(logfd, time_start, "Client disconnected. (7)");
                                errormsg("Client disconnected! Press a key to quit!", 1);
                                quit = 7;
                                break;
                            }
                            sprintf(logstring, "Read from socket %d:-%s-", sockread, socketcommbuff);
                            logmsg(logfd, time_start, logstring);
                            if (sockread == -1) {
                                if (dummymode == 0) {
                                    commandsend_lamp_on();
                                    logmsg(logfd, time_start, "Stoprobot");
                                    stoprobot(&rover, usekcommands, answer);
                                    commandsend_lamp_off();
                                }
                                logmsg(logfd, time_start, "Socket read error! (4)");
                                errormsg("Socket read error! Press a key to quit!", 1);
                                quit = 4;
                                break;
                            }
                        }

                    } else { // UDP

                        if (FD_ISSET(listenfd, &commfdset)) {
                            sockread = recvfrom(listenfd, (char *)socketcommbuff, 32, 0, (struct sockaddr *)NULL, NULL);
                            socketcommbuff[sockread] = 0;
                            sprintf(logstring, "Read UDP packet:%d:-%s-", sockread, socketcommbuff);
                            logmsg(logfd, time_start, logstring);
                            if (sockread != 12) {
                                logmsg(logfd, time_start, "UDP payload is not 12 bytes!");
                                sockread = 0;
                            }
                        }

                    }
                }

            } else {
                sockread = 0;
            }

        }

        while (sockread > 0) {
            unsigned char replymsg[16];
            unsigned int wret;

            if (remotecontrolproto == 0) { // TCP

                unsigned int sockcommi, copyi, commlen, difflen;

                for (sockcommi = 0; ((socketcommbuff[sockcommi] != '\n') && (socketcommbuff[sockcommi] != '\r') && (sockcommi < BUFFER_SIZE) && (sockcommi < sockread)); sockcommi++) {}

                if (sockcommi == BUFFER_SIZE) {
                    if (dummymode == 0) {
                        commandsend_lamp_on();
                        logmsg(logfd, time_start, "Stoprobot");
                        stoprobot(&rover, usekcommands, answer);
                        commandsend_lamp_on();
                    }
                    logmsg(logfd, time_start, "Bad message through socket - Too long (no newline found) (5)!");
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
                    logmsg(logfd, time_start, "Sent: !BADCMD!");
                    if (wret == -1) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            logmsg(logfd, time_start, "Stoprobot");
                            stoprobot(&rover, usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        logmsg(logfd, time_start, "Err: Cannot send reply to client (6).");
                        errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                        quit = 6;
                        break;
                    }
                    continue;
                }

            } else { // UDP

                sockread = 0;
                udp_packetno = (socketcommbuff[0] << 8) + socketcommbuff[1];
                if (udp_packetno > udp_lastpacketno) {
                    udp_lastpacketno = udp_packetno;
                    strncpy(receivedcommand, &socketcommbuff[2], 8);
                    receivedcommand[8] = 0;
                    // TODO checksum
                    sprintf(logstring, "Extracted command from UDP packet:-%s-", receivedcommand);
                    logmsg(logfd, time_start, logstring);
                } else {
                    continue;
                };

            }

            {
                    int badcommand = 1;

                    logmsg(logfd, time_start, "Processing command: ");
                    logmsg(logfd, time_start, receivedcommand);

                    if (strncmp(receivedcommand, "STOPZERO", 8) == 0) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            logmsg(logfd, time_start, "Stoprobot");
                            stoprobot(&rover, usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        rotate = 0;
                        speedX = 0;
                        speedY = 0;
                        badcommand = 0;
                        gettimeofday(&timestruct, NULL);
                        time_last_remotecmd_recv = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;
                        if (remotecontrolproto == 0) {
                            strncpy(replymsg, "OKZERO\r\n", 8);
                            wret = write(clientfd, replymsg, 8);
                            logmsg(logfd, time_start, "Sent: OKZERO");
                            if (wret == -1) {
                                if (dummymode == 0) {
                                    commandsend_lamp_on();
                                    logmsg(logfd, time_start, "Stoprobot");
                                    stoprobot(&rover, usekcommands, answer);
                                    commandsend_lamp_off();
                                }
                                logmsg(logfd, time_start, "Err: Cannot send reply to client (6).");
                                errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                                quit = 6;
                                break;
                            }
                        }
                    }

                    if (strncmp(receivedcommand, "ROT", 3) == 0) {
                        int newrot = 0, newrotret;
                        badcommand = 0;
                        newrotret = sscanf(&receivedcommand[3], "%d", &newrot);
                        set_new_rot_value_from_remote = 0;
                        if ((newrotret != 1) || (newrot < -10000) || (newrot > 10000)) {
                            if (remotecontrolproto == 0) {
                                strncpy(replymsg, "!BADROT!\r\n", 10);
                                wret = write(clientfd, replymsg, 10);
                                logmsg(logfd, time_start, "Sent: !BADROT!");
                                if (wret == -1) {
                                    if (dummymode == 0) {
                                        commandsend_lamp_on();
                                        logmsg(logfd, time_start, "Stoprobot");
                                        stoprobot(&rover, usekcommands, answer);
                                        commandsend_lamp_off();
                                    }
                                    logmsg(logfd, time_start, "Err: Cannot send reply to client (6).");
                                    errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                                    quit = 6;
                                    break;
                                }
                            }
                        } else {
                            rotate = newrot;
                            set_new_rot_value_from_remote = 1;
                            logmsg(logfd, time_start, "Will set newrot");
                            gettimeofday(&timestruct, NULL);
                            time_last_remotecmd_recv = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;
                        }
                    }

                    if (strncmp(receivedcommand, "SPX", 3) == 0) {
                        int newspx = 0, newspxret;
                        badcommand = 0;
                        newspxret = sscanf(&receivedcommand[3], "%d", &newspx);
                        set_new_spx_value_from_remote = 0;
                        if ((newspxret != 1) || (newspx < -10000) || (newspx > 10000)) {
                            if (remotecontrolproto == 0) {
                                strncpy(replymsg, "!BADSPX!\r\n", 10);
                                wret = write(clientfd, replymsg, 10);
                                logmsg(logfd, time_start, "Sent: !BADSPX!");
                                if (wret == -1) {
                                    if (dummymode == 0) {
                                        commandsend_lamp_on();
                                        logmsg(logfd, time_start, "Stoprobot");
                                        stoprobot(&rover, usekcommands, answer);
                                        commandsend_lamp_off();
                                    }
                                    logmsg(logfd, time_start, "Err: Cannot send reply to client (6).");
                                    errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                                    quit = 6;
                                    break;
                                }
                            }
                        } else {
                            speedX = newspx;
                            set_new_spx_value_from_remote = 1;
                            logmsg(logfd, time_start, "Will set newspx");
                            gettimeofday(&timestruct, NULL);
                            time_last_remotecmd_recv = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;
                        }
                    }

                    if (strncmp(receivedcommand, "SPY", 3) == 0) {
                        int newspy=0, newspyret;

                        badcommand = 0;
                        newspyret = sscanf(&receivedcommand[3], "%d", &newspy);
                        set_new_spy_value_from_remote = 0;
                        if ((newspyret != 1) || (newspy < -10000) || (newspy > 10000)) {
                            if (remotecontrolproto == 0) {
                                strncpy(replymsg, "!BADSPY!\r\n", 10);
                                wret = write(clientfd, replymsg, 10);
                                logmsg(logfd, time_start, "Sent: !BADSPY!");
                                if (wret == -1) {
                                    if (dummymode == 0) {
                                        commandsend_lamp_on();
                                        logmsg(logfd, time_start, "Stoprobot");
                                        stoprobot(&rover, usekcommands, answer);
                                        commandsend_lamp_off();
                                    }
                                    logmsg(logfd, time_start, "Err: Cannot send reply to client (6).");
                                    errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                                    quit = 6;
                                    break;
                                }
                            }
                        } else {
                            speedY = newspy;
                            set_new_spy_value_from_remote = 1;
                            logmsg(logfd, time_start, "Will set newspy");
                            gettimeofday(&timestruct, NULL);
                            time_last_remotecmd_recv = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;
                        }
                    }


                if ((badcommand == 1) && (remotecontrolproto == 0)) {
                    strncpy(replymsg, "!BADCMD!\r\n", 10);
                    wret = write(clientfd, replymsg, 10);
                    logmsg(logfd, time_start, "Sent: !BADCMD!");
                    if (wret == -1) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            logmsg(logfd, time_start, "Stoprobot");
                            stoprobot(&rover, usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        logmsg(logfd, time_start, "Err: Cannot send reply to client (6).");
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
                    logmsg(logfd, time_start, "Stoprobot");
                    stoprobot(&rover, usekcommands, answer);
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
        if (rover.config->has_Y_speed == 1) {
            mvprintw(commanddrawy + 2, commanddrawx + 10, "% 5d", speedY);
        }
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

        if (rover.config->has_Y_speed == 1) {
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

        gettimeofday(&timestruct, NULL);
        time_current = timestruct.tv_sec + timestruct.tv_usec / 1000000.0;

        if (remotecontrol == 1) {
            if ((time_current - time_last_remotecmd_recv) > REPEAT_TIME_SEC_REMOTECMDRECV) {
                if (remotecmd_timed_out == 0) {
                    unsigned char replymsg[16];
                    int wret;

                    logmsg(logfd, time_start, "Remotecommand timeout");
                    if (dummymode == 0) {
                        commandsend_lamp_on();
                        logmsg(logfd, time_start, "Stoprobot");
                        stoprobot(&rover, usekcommands, answer);
                        commandsend_lamp_off();
                    }
                    speedX = 0;
                    speedY = 0;
                    rotate = 0;
                    set_new_spx_value_from_remote = 0;
                    set_new_spy_value_from_remote = 0;
                    set_new_rot_value_from_remote = 0;
                    remotecmd_timed_out = 1;
                    if (remotecontrolproto == 0) { // TCP
                        strncpy(replymsg, "!NOCMST!\r\n", 10);
                        wret = write(clientfd, replymsg, 10);
                        logmsg(logfd, time_start, "Sent: !NOCMST!");
                        if (wret == -1) {
                            logmsg(logfd, time_start, "Err: Cannot send reply to client (6).");
                            errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                            perror("write()");
                            quit = 6;
                            break;
                        }
                    }
                }
            } else {
                remotecmd_timed_out = 0;
            }
        }

        if ((usekcommands == 1) && ((time_current - time_last_kcmdsent) > REPEAT_TIME_SEC_KCMDSENT)) {
            if (dummymode == 0) {
                commandsend_lamp_on();
                sprintf(logstring, "ksetXYrot: X:%d Y:%d rot:%d", speedX, speedY, rotate);
                logmsg(logfd, time_start, logstring);
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
                    sprintf(logstring, "Set robot X speed: %d", speedX);
                    logmsg(logfd, time_start, logstring);
                    setret = rover_set_X_speed(&rover, speedX, answer);
                    commandsend_lamp_off();
                    usleep(100);
                    time_last_cmdsent = time_current;
                }
                prevspeedX = speedX;
            }

            if (rover.config->has_Y_speed == 1) {
                if ( (repeatcommands == 1) || (set_new_spy_value_from_remote == 1) || (prevspeedY != speedY) ) {
// pozitiv balra - negativ jobbra
                    if (dummymode == 0) {
                        commandsend_lamp_on();
                        sprintf(logstring, "Set robot Y speed: %d", speedY);
                        logmsg(logfd, time_start, logstring);
                        setret = rover_set_Y_speed(&rover, speedY, answer);
                        commandsend_lamp_off();
                        usleep(100);
                        time_last_cmdsent = time_current;
                    }
                    prevspeedY = speedY;
                }
            }

            if ( (repeatcommands == 1) || (set_new_rot_value_from_remote == 1) || (prevrotate != rotate) ) {
// pozitiv balra forgas (100 is meg eleg lassu)
                if (dummymode == 0) {
                    commandsend_lamp_on();
                    sprintf(logstring, "Set robot rotation: %d", rotate);
                    logmsg(logfd, time_start, logstring);
                    setret = rover_set_rotation_speed(&rover, rotate, answer);
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
                    if (remotecontrolproto == 0) { // TCP
                        if (setret == 0) {
                            strncpy(replymsg, "OKSPX\r\n", 7);
                            wret = write(clientfd, replymsg, 7);
                            logmsg(logfd, time_start, "Sent: OKSPX");
                        } else {
                            strncpy(replymsg, "!ERRSPX!\r\n", 10);
                            wret = write(clientfd, replymsg, 10);
                            logmsg(logfd, time_start, "Sent: !ERRSPX!");
                        }
                    }
                }

                if (set_new_spy_value_from_remote == 1) {
                    set_new_spy_value_from_remote = 0;
                    if (remotecontrolproto == 0) { // TCP
                        if (setret == 0) {
                            strncpy(replymsg, "OKSPY\r\n", 7);
                            wret = write(clientfd, replymsg, 7);
                            logmsg(logfd, time_start, "Sent: OKSPY");
                        } else {
                            strncpy(replymsg, "!ERRSPY!\r\n", 10);
                            wret = write(clientfd, replymsg, 10);
                            logmsg(logfd, time_start, "Sent: !ERRSPY!");
                        }
                    }
                }

                if (set_new_rot_value_from_remote == 1) {
                    set_new_rot_value_from_remote = 0;
                    if (remotecontrolproto == 0) { // TCP
                        if (setret == 0) {
                            strncpy(replymsg, "OKROT\r\n", 7);
                            wret = write(clientfd, replymsg, 7);
                            logmsg(logfd, time_start, "Sent: OKROT");
                        } else {
                            strncpy(replymsg, "!ERRROT!\r\n", 10);
                            wret = write(clientfd, replymsg, 10);
                            logmsg(logfd, time_start, "Sent: !ERRROT!");
                        }
                    }
                }

                if (remotecontrolproto == 0) { // TCP
                    if (wret == -1) {
                        if (dummymode == 0) {
                            commandsend_lamp_on();
                            logmsg(logfd, time_start, "Stoprobot");
                            stoprobot(&rover, usekcommands, answer);
                            commandsend_lamp_off();
                        }
                        logmsg(logfd, time_start, "Err: Cannot sent reply to client (6).");
                        errormsg("Cannot send reply to client! Connection lost? Press a key to quit!", 1);
                        quit = 6;
                        break;
                    }
                }

            }

        }

    }

    // ncurses close
    endwin();

    if (remotecontrol == 1) {
        unsigned char replymsg[24];
        int wret;

        if (remotecontrolproto == 0) { // TCP
            strncpy(replymsg, "Closing. Byebye!\r\n", 18);
            wret = write(clientfd, replymsg, 17);
            if (wret == -1) {
                if (dummymode == 0) {
                    logmsg(logfd, time_start, "Stoprobot");
                    stoprobot(&rover, usekcommands, answer);
                }
                printf("Could not say goodbye to client... Connection lost?");
                perror("write()");
                quit = 6;
            }
            close(clientfd);
        }

        close(listenfd);
    }

    switch (quit) {
        case 2: printf("Fatal error happened while reading memmap!\n"); break;
        case 3: printf("Failed to read memmap correctly (invalid length)!\n"); break;
        case 4: printf("Error while reading from socket()!\n"); break;
        case 5: printf("Bad message received through socket - Too long (no newline found)!\n"); break;
        case 6: printf("Could not reply to client! Connection was lost maybe?\n"); break;
        case 7: printf("Client disconnected!\n"); break;
    }

    if (got_sigpipe == 1) { printf("Got SIGPIPE! Connection to client was lost maybe?\n"); }

    if (dummymode == 0) {
        printf("Setting X+Y+Rot speed to zero.\n");
        stoprobot(&rover, usekcommands, answer);
        printf("Disabling motors on main controller.\n");
        rover_disable_motors(&rover, rover.regs->controller_addr_main, answer);
        if (rover.config->has_second_controller == 1) {
            printf("Disabling motors on second controller.\n");
            rover_disable_motors(&rover, rover.regs->controller_addr_second, answer);
        }
    }

    logmsg(logfd, time_start, "Exit");

    close(logfd);

    return 0;

}
