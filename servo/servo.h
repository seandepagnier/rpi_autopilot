/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

int servo_open(const char *device);
void servo_capabilities(int servo);
void servo_write(int servo, int command);
int servo_current(int servo, float *current);
