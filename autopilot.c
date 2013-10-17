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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>

#include <stdint.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "calibration.h"
#include "rotate.h"
#include "quaternion.h"
#include "vector.h"

#include "MadgwickAHRS/MadgwickAHRS.h"

/* configuration */

const char device[] = "/dev/i2c-1";
const char servo_device[] = "/dev/ttyUSB0";

float desired_heading = 120;
static float pid_gains[18] =
// P   R   Y   P'  R'  Y'
{  0,  0,  .1,  0,  0,  .18,  // P
   0,  0,  0,  0,  0,  0,   // I
   0,  0,  0,  0,  0,  0};  // D

int servo_command_period = 1; /* 2 seconds */
int servo_direction = -1; /* physical configuration */
float default_beta = .1;

float gyro_lp = .999;
float gyro_scale = 900;

/* end of configuration */


int file;
int8_t buf[12];

int servo;

#define BMA180 0x40
#define ITG3200 0x68
#define BMP085 0x77
#define HMC5883 0x1e

enum {ACCELX, ACCELY, ACCELZ, GYROX, GYROY, GYROZ, MAGX, MAGY, MAGZ, BARO};
static float of_bias[10] = {0, 0, 0, -.05, .11, .03, .24, .63, -.5, 0};

int engauged = 0;
static float states[18];
float command;

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

void i2c_read(uint8_t addr, int len)
{
    buf[0] = addr;
    write(file, buf, 1);
    read(file, buf, len);
}

void i2c_write_byte(uint8_t addr, uint8_t value)
{
    buf[0] = addr;
    buf[1] = value;
    write(file, buf, 2);
}

void probe_devices()
{
    if(ioctl(file, I2C_SLAVE, BMA180) == 0) {
        i2c_read(0x00, 2); /* identifier */
        if(buf[0] == 0x03 && buf[1] == 0x14) {
            printf("found bma180\n");
        }
    }

    if(ioctl(file, I2C_SLAVE, ITG3200) == 0) {
        i2c_read(0x00, 1);
        if(buf[0] == ITG3200) {
            printf("found itg3200\n");
            i2c_write_byte(0x15, 0x07); /* sample rate divider 1khz / (x + 1) */
            i2c_write_byte(0x16, 0x1d); /* 2000 deg/sec range and 10hz low pass filter */
            i2c_write_byte(0x3e, 0x01); /* clock source set to PLL with x gyro ref */
        }
    }

    if(ioctl(file, I2C_SLAVE, HMC5883) == 0) {
        i2c_read(0x0a, 3);
        if(buf[0] == 'H' && buf[1] == '4' && buf[2] == '3') {
            printf("found hmc5883\n");
        }
    }

    if(ioctl(file, I2C_SLAVE, BMP085) == 0) {
        printf("found bmp085\n");
    }
}

struct timeval start;
float now()
{
    struct timeval current;
    gettimeofday(&current, NULL);
    return current.tv_sec - start.tv_sec + (current.tv_usec - start.tv_usec)/1e6;
}

    float dof[10], cof[10], of[10];
void read_sensors()
{
    if(ioctl(file, I2C_SLAVE, BMA180) < 0) {
        fprintf(stderr, "Failed to aquire bus address\n");
        exit(1);
    }

    float s;
    int i;

    i2c_read(0x02, 6); /* read data */
    if(buf[0] & 1 && buf[2] & 1 && buf[4] & 1) {
        s = 16384;
        for(i=0; i<3; i++) {
            dof[i] += (float)(((int16_t)buf[2*i+1]<<8) + (uint8_t)buf[2*i]) / s;
            cof[i]++;
        }
    } else
        printf("accel miss\n");
    
    if(ioctl(file, I2C_SLAVE, ITG3200) < 0) {
        fprintf(stderr, "Failed to aquire bus address\n");
        exit(1);
    }

    i2c_read(0x1a, 1); /* read status byte */
    if(buf[0] & 1) {
//            s = 2048;
                    
        i2c_read(0x1d, 6); /* read data */
        for(i=0; i<3; i++) {
            dof[3+i] += (float)(((int16_t)buf[2*i+0]<<8) + (uint8_t)buf[2*i+1]) / gyro_scale;
            cof[3+i]++;
        }
    } else
        printf("gyro miss\n");
                    
                    
    if(ioctl(file, I2C_SLAVE, HMC5883) < 0) {
        fprintf(stderr, "Failed to aquire bus address\n");
        exit(1);
    }
                
    /* read data */
    i2c_read(0x09, 1); /* read status byte */
    if(buf[0] & 1) {
        s=512;

        i2c_read(0x03, 6); /* read data */
        for(i=0; i<3; i++) {
            int ind[3] = {0, 4, 2};
            dof[6+i] += (float)((int)((int16_t)buf[ind[i]]<<8) + (uint8_t)buf[ind[i]+1]) / s;

            cof[6+i]++;
        }

//            i2c_write_byte(0x09, 0x00);
    } else
        printf("mag miss\n");

    i2c_write_byte(0x02, 0x01); /* request conversion */


    if(ioctl(file, I2C_SLAVE, BMP085) < 0) {
        fprintf(stderr, "Failed to aquire bus address\n");
        exit(1);
    }

    i2c_read(0xf6, 3);
    s = 1<<24;
    dof[9] += (float)(((int32_t)buf[0]<<24) +
                      ((int32_t)buf[1]<<16) +
                      ((int32_t)buf[2])) / s;  cof[9]++;

    i2c_write_byte(0xf4, 0xf4); /* request conversion */
}

int main()
{
    beta = default_beta;

    gettimeofday(&start, NULL);

    /* configure serial port */
    char cmd[256] = "";
    strcat(cmd, "stty -F ");
    strcat(cmd, servo_device);
    strcat(cmd, " 9600 ignbrk -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 1 time 5");
    system(cmd);

    if(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set stdin non blocking\n");
        exit(1);
    }
    setvbuf(stdin, 0, _IONBF, 0);

    if((servo = open(servo_device, O_RDWR)) < 0)
        printf("failed to open '%s' to control servo\n", servo_device);
    else if(fcntl(servo, F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set '%s' non blocking\n", servo_device);
        exit(1);
    }

    if((file = open(device, O_RDWR)) < 0)
        fprintf(stderr, "failed to open: %s\n", device);

    init_mag_calibration();
    probe_devices();

    int i;

    int filter_samples=0, command_samples=0;
    float pitchrate=0, rollrate=0, yawrate=0;
    const int filter_sample_count = 10, command_sample_count = 100;
    for(i=0; i<10; i++)
        of[i] = dof[i] = cof[i] = 0;

    for(;;) {
        if(file != -1) {
//            if(filter_samples == 0)
                read_sensors();
        }

        if(++filter_samples >= filter_sample_count) {
            filter_samples = 0;

            /* average results */
            for(i=0; i<10; i++) {
                if(cof[i] == 0)
                    goto skip;
                of[i] = dof[i] / cof[i];
                dof[i] = cof[i] = 0;
            }

            /* long lowpass gyros to cancel bias */
            for(i=3; i<6; i++)
                of_bias[i] = lp_filter(of_bias[i], of[i], gyro_lp);

            for(i=0; i<10; i++)
                of[i] -= of_bias[i];

            /* orient toward initial mag reading
               to avoid large yaw error at start */
            static int initial_mag = 1;
            if(initial_mag) {
                initial_mag = 0;
                float q[4], a[3] = {of[0], of[1], of[2]}, g[3] = {0, 0, 1};
                float m[3] = {of[6], of[7], of[8]};
                rotateoutmag(a, m);
                angvec2quat(q, atan2(m[1], m[0]), g);
                q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
            }
                    
#if 1
            MadgwickAHRSupdate(of[3], of[4], -of[5], of[0], of[1], of[2], of[6], -of[7], -of[8]);
#else
            MahonyAHRSupdate(of[3], of[4], of[5], of[0], of[1], of[2], of[6], of[7], of[8]);
#endif

#if 1
            float q[4] = {q0, q1, q2, q3};
            float pitch, roll, yaw;
            quaternion_pitch_roll_yaw(q, &pitch, &roll, &yaw);

            rollrate = lp_filter(rollrate, rad2deg(of[3]), .9);
            pitchrate = lp_filter(pitchrate, rad2deg(of[4]), .9);
            yawrate = lp_filter(yawrate, -rad2deg(of[5]), .9);

            // PID Filter
            /* compute position, integral and derivative from all 6 states */
            float heading_error = heading_resolve(yaw - desired_heading);
            float inputstates[6] = {pitch, roll, heading_error, pitchrate, rollrate, yawrate};

            for(i=0; i<6; i++) {
                states[i+6] += inputstates[i] / filter_sample_count;
                states[i+12] = (inputstates[i] - states[i])*filter_sample_count;
                states[i] = inputstates[i];
            }


            print_value(states[2]);
            print_value(states[5]);

            print_value(states[14]);

#if 1
                float a[3] = {of[0], of[1], of[2]}, g[3] = {0, 0, 1};
                float m[3] = {of[6], of[7], of[8]};
                rotateoutmag(a, m);
                print_value(heading_resolve(rad2deg(atan2(m[1], m[0])) - desired_heading));
#endif
            printf("\n");
            
#endif
        }
    skip:;

        if(++command_samples >= command_sample_count) {
            command_samples = 0;

            float q[4] = {q0, q1, q2, q3};
            float pitch, roll, yaw;
            quaternion_pitch_roll_yaw(q, &pitch, &roll, &yaw);


#if 0
            print_value(of_bias[3]);
            print_value(of_bias[4]);
             print_value(of_bias[5]);
#endif
//            print_value(yaw);


            command = lp_filter(command, pid_command(), .8);

            int n = now();
            char scommand[128] = "";
            if(engauged) {
                float scaled_command = servo_direction*1000*command;
                if(scaled_command > 1000)
                    scaled_command = 1000;
                if(scaled_command < -1000)
                    scaled_command = -1000;

                /* don't command initially because the state is inaccurate */                
                if(n > 5 && (n%servo_command_period)==0) {
                    sprintf(scommand, "%d\r\n", (int)(scaled_command));
                    write(servo, scommand, strlen(scommand));
                }
            }

            while((read(servo, buf, 1)) == 1) {
                switch(buf[0]) {
                case 'k': /* accepted command */
                    break;
                default: /* unknown */
                    break;
                }
            }
#if 0
            print_value(heading_error);
            printf(" ");
            scommand[strlen(scommand)-1] = '\0';
            printf(scommand);
#endif

#if 0
            print_value(of[MAGX]);
            print_value(of[MAGY]);
            print_value(of[MAGZ]);
#endif
            //       printf("\n");

            float magstate[4];
            if(calibrate_mag(of + 6, magstate))
                for(i=0; i<3; i++)
                    of_bias[MAGX+i] += magstate[i];
#if 0
            printf("biases: ");
            for(i=0; i<9; i++)
                print_value(of_bias[i]);
#endif


            int print = 0;
            while((read(STDIN_FILENO, buf, 1)) == 1) {
                switch(buf[0]) {
                case 'q': exit(1);
                case '+': case '=': desired_heading++; break;
                case '_': case '-': desired_heading--; break;
                case '0': desired_heading = yaw; break;
                case 'a': engauged = !engauged; break;
                }
                print = 1;
            }
            if(print)
                printf("heading: %.2f %d\n", desired_heading, engauged);
        }

        fflush(stdout);

        /* run at approximately 100 hz */
        struct timespec ts = {0, 1e7};
        nanosleep(&ts, NULL);
    }
}
