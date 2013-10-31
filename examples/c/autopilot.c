/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
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

#include "imu.h"
#include "servo.h"

#include "calibration.h"
#include "rotate.h"
#include "quaternion.h"
#include "vector.h"

float desired_heading = 120;
static float pid_gains[18] =
// P   R   Y   P'  R'  Y'
{  0,  0,  .1,  0,  0, .16,  // P
   0,  0,  0,  0,  0,  0,    // I
   0,  0,  0,  0,  0,  0};   // D


int engauged = 0;
static float states[18];
float command;

void print_value(float v)
{
    printf("%c%05.2f ", v < 0 ? '-' : '+', fabs(v));
}

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

int yaw_dampening = 0;
void yaw_dampen(int ind)
{
    #define logsize 100
    static float log[SENSOR_COUNT][logsize];
    static int logstart[SENSOR_COUNT], logend[SENSOR_COUNT];
    log[ind][logstart[ind]--] = states[ind];
    if(logstart[ind] < 0)
        logstart[ind] = logsize-1;
    if(logstart[ind] == logend[ind])
        logend[ind]--;
    if(logend[ind] < 0)
        logend[ind] = logsize-1;

//    printf("%d %d:", logstart[ind], logend[ind]);
    int i, c=0;
    float curlog[logsize];

//    for(i=0; i<logsize; i++)
//        print_value(log[ind][i]);
//    printf("\n");

    for(i = logstart[ind]; i!= logend[ind]; i++) {
        if(i == logsize)
            i = -1;
        else {
            curlog[c] = log[ind][i];
            c++;
        }
    }

#if 1
    float X[4], r;
    r = CalcBestLine(curlog, c, X);
//        printf("line %f %f, %f\n", X[0], X[1], r);


    X[0] = 1, X[1] = 10, X[2] = 0, X[3] = 0;

        int iters;
        for(iters = 0; iters < 2; iters++) {
            r = CalcBestSine(curlog, c, X);
        }

        if(r) {
//            float x = c;
//            float yd = states[ind] - X[2]*sin(X[3]*x + X[4]);
//            print_value(yd);
//            if(fabsf(r) < 1 && X[1] > 3 && X[1] < 8 && fabsf(X[3]) < 10)
            {
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

#endif
}

void config(int op)
{
/*
    const char *in[3] = {"P", "I", "D"};

  int i;
    for(i=0; i<3; i++)
        config_float_vector(op, f, in[i], pid_gains + i*6, 6);

    // scale factor and polarity of servo configuration,
    // servo is controlled by an avr which takes +- 1000
    config_float(op, f, "servo_scale", &servo_scale);
    config_float(op, f, "min_hardover_period", &min_hardover_period);
    config_float_vector(op, f, "alignment", alignment, 4);
*/
}

struct timeval start;
float now()
{
    struct timeval current;
    gettimeofday(&current, NULL);
    return current.tv_sec - start.tv_sec + (current.tv_usec - start.tv_usec)/1e6;
}

int main()
{
    imu_init("~/.rpi_ap/imu");
    int servo = servo_open("/dev/ttyUSB0");

    /* for keyboard control */
    if(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set stdin non blocking\n");
        exit(1);
    }
    setvbuf(stdin, 0, _IONBF, 0);

    /* record start time */
    gettimeofday(&start, NULL);

    for(;;) {
        float X[6];
        imu_orientation(X);
        imu_rate(X+3);

        int i;
        for(i=0; i<6; i++)
            print_value(X[i]);
        printf("\n");
        
        // PID Filter
        /* compute position, integral and derivative from all 6 states */

        const float hz = 10; /* run about 10 hz */
        for(i=0; i<6; i++) {
            states[i+6] += X[i] / hz;
            if(states[i+6] > 1) states[i+6] = 1;
            if(states[i+6] < -1) states[i+6] = -1;

            states[i+12] = (X[i] - states[i])*hz;

            if(i == 2)
                states[i] = heading_resolve(X[2] - desired_heading);
            else
                states[i] = X[i];
        }

        command = pid_command();

        int n = now();

        /* don't command initially because the state is inaccurate */                
        if(n > 5 && engauged)
            servo_write(servo, command);

        servo_read();

        /* read from stdin */
        int print = 0;
        char buf[1];
        while((read(STDIN_FILENO, buf, 1)) == 1) {
            switch(buf[0]) {
            case 'q': exit(1);
            case '+': case '=': desired_heading++; break;
            case '_': case '-': desired_heading--; break;
            case '0': desired_heading += states[2]; break;
            case 'a': engauged = !engauged; break;
            case 'y': yaw_dampening = !yaw_dampening; break;
            case 'l': imu_level(); break;
            }
            print = 1;
        }
        if(print)
            fprintf(stderr, "heading: %.2f %d %d\n", desired_heading, engauged, yaw_dampening);
   
        fflush(stdout);
    
        /* run at approximately 10 hz */
        struct timespec ts = {0, 1e8};
        nanosleep(&ts, NULL);
    }
}

