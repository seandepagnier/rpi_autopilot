#include <stdio.h>
#include <stdlib.h>

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "config.h"

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
