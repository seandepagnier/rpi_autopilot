/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "servo.h"

#include "config.h"

float servo_scale = -1000; /* physical configuration */
float min_hardover_period = 2;

float servo_command_hz = 10;

static void config(int op)
{
    FILE *f = config_open(op, "servo");

    // scale factor and polarity of servo configuration,
    // servo is controlled by an avr which takes +- 1000
    config_float(op, f, "servo_scale", &servo_scale);

    config_float(op, f, "min_hardover_period", &min_hardover_period);

    fclose(f);
}

int servo_open(const char *device)
{
    /* configure serial port to drive motor */
    char cmd[256] = "";
    strcat(cmd, "stty -F ");
    strcat(cmd, device);
    strcat(cmd, " 9600 ignbrk -icrnl -opost -onlcr -isig -icanon "
           "-iexten -echo -echoe -echok -echoctl -echoke min 1 time 5");
    system(cmd);

    int servo;
    if((servo = open(device, O_RDWR)) < 0)
        printf("failed to open '%s' to control servo\n", device);
    else if(fcntl(servo, F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set '%s' non blocking\n", device);
        exit(1);
    }
    return servo;
}

void servo_write(int servo, int command)
{
    char scommand[128] = "";
    float scaled_command = servo_scale*command;
    if(scaled_command > fabsf(servo_scale))
        scaled_command = servo_scale;
    if(scaled_command < -fabsf(servo_scale))
        scaled_command = -servo_scale;
    
    static int lsc;
    int max_command = 2*fabsf(servo_scale) / min_hardover_period * servo_command_hz / 100;
    if(scaled_command - lsc > max_command)
        scaled_command = max_command + lsc;
    if(scaled_command - lsc < -max_command)
        scaled_command = -max_command + lsc;
    lsc = scaled_command;

    FILE *s = fdopen(servo, "w");
    fprintf(s, "c%d\r\n", (int)(scaled_command));
}

int servo_read(int servo)
{
    char buf[10];
    while((read(servo, buf, 1)) == 1) {
        switch(buf[0]) {
        case 'e': /* stall */
        case 'o': /* overflow */
        case 'i': /* invalid */
        case 'f': /* out of range */
        case 'n': /* not accepting commands */
            printf("servo sent: %c\n", buf[0]);
            break;
        case 'k': /* accepted command */
            break;
        case 'P': /* hold button port */
        case 'p': /* button port */
        case 'S': /* hold button starboard */
        case 's': /* button starboard */
            break;
        case 'm': /* monitor reading */
            break;
        default: /* unknown */
            break;
        }
    }
    return 0;
}

int servo_set_max_current(int servo, float current)
{
    FILE *s = fdopen(servo, "w");
    fprintf(s, "s%d\r\n", (int)(current*10.0));
}
