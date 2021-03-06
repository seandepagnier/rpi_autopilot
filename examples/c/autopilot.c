/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/*
  This basic autopilot for sailboats reads from i2c sensors and via and avr,
  commands brushed or brushless motors or servos using pwm output or via relays
  connected to io pins.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdint.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include <ncurses.h>

#include "imu.h"
#include "calibration.h"
#include "rotate.h"
#include "quaternion.h"
#include "vector.h"

#include "servo.h"

#include "config.h"

#define UPDATE_RATE 10 /* 10 hz */

float desired_heading = 120;
enum {PITCH, ROLL, YAW, PITCH_RATE, ROLL_RATE, YAW_RATE};
enum {POSITION, INTEGRAL=6, DERIVATIVE=12};
static float pid_gains[18] =
// P   R   Y   P'  R'  Y'
{  0,  0, .1,  0,  0, .16,   // P
   0,  0,  0,  0,  0,  0,    // I
   0,  0,  0,  0,  0,  0};   // D


int engauged = 0;
static float states[18];
float command;

float heel; /* average roll over a longer period */

float pid_command()
{
    int i;
    float total = 0;
    for(i=0; i<18; i++) {
        total += states[i] * pid_gains[i];
#if 0
        if(pid_gains[i])
            printf("pid%d: %.3f ", i, states[i]*pid_gains[i]);
#endif
    }
    return total;
}

int sensorlogpos, sensorlogready;
float sensorlog[10*UPDATE_RATE][6]; /* 10 second memory */
#define sensorlogsize (sizeof sensorlog) / (sizeof *sensorlog)

void analyze_state(float state[6])
{
    memcpy(sensorlog[sensorlogpos++], state, sizeof state);
    if(sensorlogpos == sensorlogsize) {
        sensorlogpos = 0;
        sensorlogready = 1;
    }

    if(sensorlogready) {
        /* log in transposed table, not ringbuffer */
        float curlog[6][sensorlogsize];
        
        int s, i, j;
        for(s=0; s<6; s++) {
            for(j = 0, i = sensorlogpos; i<sensorlogsize; i++, j++)
                curlog[s][j] = sensorlog[i][s];
            for(i = 0; i != sensorlogpos; i++, j++)
                curlog[s][j] = sensorlog[i][s];

            float X[4], r;
            r = CalcBestLine(curlog[s], sensorlogsize, X);
            printf("line %d %f %f, %f\n", s, X[0], X[1], r);

            X[0] = 1, X[1] = 10, X[2] = 0, X[3] = 0;

            int iters;
            for(iters = 0; iters < 5; iters++) {
                r = CalcBestSine(curlog[s], sensorlogsize, X);
            }

            if(r) {
//            float x = c;
//            float yd = states[ind] - X[2]*sin(X[3]*x + X[4]);
//            print_value(yd);
//            if(fabsf(r) < 1 && X[1] > 3 && X[1] < 8 && fabsf(X[3]) < 10)
#if 0
            printf("quadratic sine ");
            print_value(X[0]);
            print_value(X[1]);
            print_value(X[2]);
            print_value(X[3]);
            print_value(r);
            print_value(yd);
#elif 0
            print_value(states[ind]);
            if(fabsf(X[3]) < 10)
                print_value(X[0]);
            else
                print_value(0);
#endif
//            printf("\n");
            }
        }
    }
}

static void config(int op)
{
    FILE *f = config_open(op, "autopilot");

    const char *in[3] = {"P", "I", "D"};
    int i;
    for(i=0; i<3; i++)
        config_float_vector(op, f, in[i], pid_gains + i*6, 6);

    fclose(f);
}

void read_input()
{
    for(;;)
        switch(getch()) {
        case -1: return;
        case 'q': exit(1);
        case '+': case '=': desired_heading++; break;
        case '_': case '-': desired_heading--; break;
        case '0': desired_heading += states[YAW]; break;
        case 'a': engauged = !engauged; break;
        case 'l': imu_level(); break;
        }

//    fprintf(stderr, "heading: %.2f %d\n", desired_heading, engauged);
}

void setup_ncurses()
{
    initscr();
    raw();
    cbreak();
    nodelay(stdscr, 0);
    noecho();

    /* for nonblocking control */
    if(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set stdin non blocking\n");
        exit(1);
    }
}

void print_valuew(float v)
{
    printw("%c%05.2f ", v < 0 ? '-' : '+', fabs(v));
}

struct servo *servo = NULL;

void cleanup()
{
    servo_close(servo);
    erase();
    if(isendwin() == 0)
        endwin();
}

int main()
{
    imu_init();
    servo = servo_open("/dev/ttyUSB0");

    atexit(cleanup);

    servo_set_mode(servo, 0, "POSITION");

    setup_ncurses();
    printw("Autopilot ncurses Control Interface\n"
           "Keys include:\n"
           "\t+/- - set desired heading\n"
           "\t0 - desired heading to current heading\n"
           "\ta - engauged/disengauge autopilot\n"
           "\tl - level imu\n");

    for(;;) {
        float X[6];
        imu_orientation(X);
        imu_rate(X+3);

        // PID Filter
        /* compute position, integral and derivative from all 6 states */

        const float hz = 10; /* run about 10 hz */
        int i;
        float heading;
        for(i=0; i<6; i++) {
            if(i == YAW) { /* set yaw to yaw error */
                heading = X[YAW];
                states[i] = heading_resolve(heading - desired_heading);
            } else
                states[i] = X[i];

            /* integrate with saturation */
            states[i+INTEGRAL] += X[i] / hz;
            if(states[i+INTEGRAL] > 1) states[i+INTEGRAL] = 1;
            if(states[i+INTEGRAL] < -1) states[i+INTEGRAL] = -1;

            /* differentiate */
            states[i+DERIVATIVE] = (X[i] - states[i])*hz;
        }

        /* analyze the states */
        
        enum {PORT, STARBOARD, };
        heel = lp_filter(heel, states[ROLL], .99);

        command = pid_command();

        int n = imu_runtime();

        /* don't command initially because the state is inaccurate */                
        if(n > 5 && engauged)
            servo_command(servo, 0, command);

        read_input();
        mvprintw(6, 0, "heading:        ");
        print_valuew(heading);
        mvprintw(6, 40, "rate:         ");
        print_valuew(states[YAW_RATE]);
        mvprintw(7, 0, "desired heading:          ");
        print_valuew(desired_heading);
        mvprintw(7, 40, "heading error:        ");
        print_valuew(states[YAW]);

        const char *status = servo_read_status(servo);
        if(status)
            mvprintw(8, 0, "Status: %s   ", status);

        /* run at approximately 10 hz */
        struct timespec ts = {0, 1e8};
        nanosleep(&ts, NULL);
    }
}
