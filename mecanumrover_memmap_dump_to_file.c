/*
    NLAB-MecanumCommlib for Linux, a simple library to control VStone MecanumRover 2.1 / VStone MegaRover 3
    by David Vincze, vincze.david@webcode.hu
    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021-2022
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

    int ret, fd;
    struct roverstruct rover;

    ret = rover_read_full_memmap(rover.memmap_main, CONTROLLER_ADDR_MAIN, &rover);

    fd = open("memmap_dump.dat", O_WRONLY | O_CREAT | O_TRUNC);
    if (fd == -1) {
        perror("open(memmap_dump.dat)");
        exit(1);
    }
    if (write(fd, rover.memmap_main, 384) != 384) {
        printf("write() error");
        exit(1);
    }
    close(fd);

    return 0;

}
