/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

struct servo
{
    FILE *file;

    /* servo capabilities */
    int motor_count;
    int button_count;
    int buzzer;   
};


struct servo *servo_open(const char *device);
void servo_close(struct servo *servo);
const char *servo_read_status(struct servo *servo);
void servo_set_mode(struct servo *s, int motor, const char *mode);
void servo_command(struct servo *s, int motor, float command);
