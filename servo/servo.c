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
#include <time.h>

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

void servo_set_mode(struct servo *s, int motor, const char *mode)
{
    if(s)
        fprintf(s->file, "!SETVAR %d MODE %s\n", motor, mode);
}

void servo_command(struct servo *s, int motor, float command)
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

    if(s)
        fprintf(s->file, "!SETVAR %d CMD %d\r\n", motor, (int)(scaled_command));
}

static int cmdcmp(const char *cmd, const char *s, char **endptr)
{
    int len = strlen(cmd);
    if(endptr) {
        *endptr = (char*)s+len;
        while(**endptr == ' ') *endptr++;
    }
    return !strncmp(cmd, s, len);
}

struct servo *servo_open(const char *device)
{
    /* configure serial port to drive motor */
    char cmd[256] = "";
    strcat(cmd, "stty -F ");
    strcat(cmd, device);
    strcat(cmd, " 9600 ignbrk -icrnl -opost -onlcr -isig -icanon "
           "-iexten -echo -echoe -echok -echoctl -echoke min 1 time 5");
    system(cmd);

    FILE *file;
    if(!(file = fopen(device, "rw"))) {
        printf("failed to open '%s' to control servo\n", device);
        return NULL;
    }

    struct servo *servo = malloc(sizeof *servo);
    if(!servo)
        return NULL;

    servo->motor_count = servo->button_count = servo->buzzer = 0;
    servo->file = file;

    if(fcntl(fileno(servo->file), F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set '%s' non blocking\n", device);
        exit(1);
    }
    setlinebuf(servo->file);

    fprintf(servo->file, "!GETCAP\r\n");

    /* wait for it */
    struct timespec ts = {0, 1e8};
    nanosleep(&ts, NULL);

    /* read capabilities */
    char s[128], *e, *f;
    while(fgets(s, sizeof s, servo->file))
        if(cmdcmp("!CAP", s, &e))
            while(*e) {
                while(*e == ' ') e++;
                if(cmdcmp("MOTORS=", e, &f))
                    servo->motor_count = strtol(f, &e, 10);
                else if(cmdcmp("BUTTONS=", e, &f))
                    servo->button_count = strtol(f, &e, 10);
                else if(cmdcmp("BUZZER", e, &f))
                    servo->buzzer = 1, e = f;
                while(*e != ' ') e++;
            }

    return servo;
}

void servo_close(struct servo *servo)
{
    if(!servo)
        return;

    for(int m=0; m<servo->motor_count; m++)
        servo_set_mode(servo, m, "IDLE");
    fclose(servo->file);
    free(servo);
}

const char *servo_read_status(struct servo *servo)
{
    if(!servo)
        return NULL;

    static char laststatus[128];
    int status = 0;
    char s[128], *e;
    while(fgets(s, sizeof s, servo->file)) {
        if(cmdcmp("!STATUS", s, &e)) {
            strcpy(laststatus, e);
            status = 1;
        }
    }

    if(status)
        return laststatus;
    return NULL;
}
