/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <unistd.h>

#include "imu.h"

#define UPDATE_RATE 10 /* 10 hz */

void read_stdin()
{
    /* read from stdin */
    int print = 0;
    char buf[1];
    while((read(STDIN_FILENO, buf, 1)) == 1) {
        switch(buf[0]) {
        case 'q': exit(1);
        case 'l': imu_level(); break;
        }
    }
}

int main()
{
    imu_init();

    /* for keyboard control */
    if(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set stdin non blocking\n");
        exit(1);
    }
    setvbuf(stdin, 0, _IONBF, 0);

    for(;;) {
        read_stdin();

        float X[6];
        imu_orientation(X);
        imu_rate(X+3);

        int i;
        for(i=0; i<6; i++)
            print_value(X[i]);
        printf("\n");

        struct timespec ts = {0, 1e9/UPDATE_RATE};
        nanosleep(&ts, NULL);
    }
}
