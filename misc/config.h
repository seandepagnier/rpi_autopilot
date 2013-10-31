/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

enum {READ, WRITE};
FILE *config_open(int op, const char *config_name);
int config_string(int op, FILE *f, const char *name, char *str, int maxlen);
void config_float_vector(int op, FILE *f, const char *name, float *value, int c);
void config_float(int op, FILE *f, const char *name, float *value);

