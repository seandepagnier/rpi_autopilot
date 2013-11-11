/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#define __USE_GNU
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <math.h>

#include <errno.h>

#include "config.h"


FILE *config_open(int op, const char *config_name)
{
    char *defpath;
    asprintf(&defpath, "%s/.rpi_ap", getenv("HOME"));

    char *path = get_current_dir_name();

    FILE *f = NULL;
    /* cd to config dir */
    if(chdir(defpath) == -1) {
        if(errno == ENOENT) {
            mkdir(defpath, 0755);
            if(chdir(defpath) == -1)
                goto abort;
        } else
            goto abort;
    }

    
    
    if(!(f = fopen(config_name, op == READ ? "r" : "w")))
        fprintf(stderr, "failed to open config: %s\n", config_name);

    /* reset working directory */
    chdir(path);
    free(path);

abort:
    free(defpath);
    return f;
}

/* persistant data, like calibration and configuration settings */
int config_string(int op, FILE *f, const char *name, char *str, int maxlen)
{
    if(op == WRITE) {
        fprintf(f, "%s %s\n", name, str);
        return 1;
    }

    fseek(f, 0, SEEK_SET);
    char buffer[256];
    int pos = strlen(name);
    while(fgets(buffer, sizeof buffer, f)) {
        if(buffer[strlen(buffer)-1] == '\n')
            buffer[strlen(buffer)-1] = '\0';
        if(!strncmp(buffer, name, pos)) {
            int len = maxlen, buflen = sizeof buffer - pos - 1;
            if(len > buflen)
                len = buflen;
            while(buffer[pos] == ' ') pos++;
            strncpy(str, buffer + pos, len);
            return 1;
        }
    }

    return 0;
}

void config_float_vector(int op, FILE *f, const char *name, float *value, int c)
{
    int i;
    if(op == WRITE) {
        fprintf(f, "%s", name);
        for(i = 0; i<c; i++)
            fprintf(f, " %g", value[i]);
        fprintf(f, "\n");
        return;
    }

    char buffer[128];
    if(config_string(op, f, name, buffer, sizeof buffer)) {
        char *ptr = buffer, *endptr;
        for(i=0; i<c; i++) {
            float v = strtof(ptr, &endptr);
            if(ptr != endptr)
                value[i] = v;
            else
                fprintf(stderr, "invalid number in config file: %s %d %d\n", buffer, ptr-buffer, endptr-buffer);
            ptr = endptr;
        }
    }
}

void config_float(int op, FILE *f, const char *name, float *value)
{
    config_float_vector(op, f, name, value, 1);
}
