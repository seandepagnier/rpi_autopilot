/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

struct servo
{
    FILE *f;
    int motor_count, button_count;
    int mode_count;
    char **modes;
};    

struct servo *servo_open(const char *device);

void servo_write(int servo, int command);
int servo_setmode(struct servo *servo, int motor, float maxcurrent, const char *mode);
int servo_cmd(struct servo *servo, int motor, float command);
int servo_buzz(struct servo *servo, int n);
int servo_motor(struct servo *servo,
