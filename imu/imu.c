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

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pthread.h>

#include "MadgwickAHRS/MadgwickAHRS.h"

#include "imu.h"
#include "vector.h"
#include "quaternion.h"
#include "rotate.h"
#include "calibration.h"
#include "config.h"

#define BMA180 0x40
#define ITG3200 0x68
#define BMP085 0x77
#define HMC5883 0x1e

int file;
int8_t buf[12];

float dof[SENSOR_COUNT], cof[SENSOR_COUNT], of[SENSOR_COUNT];
float inertial_state[6];

/* configuration */
static float of_bias[SENSOR_COUNT] = {0, 0, 0, -.05, .11, .03, .24, .63, -.5, 0};

char i2c_device[] = "/dev/i2c-1";

float alignment[4] = {1, 0, 0, 0}; /* quaternion */

float gyro_bias_lowpass = .995;
float gyro_scale = 2000;

float desired_heading;
const int filter_sample_count = 5;

static char *config_name="imu";

//pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

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

static void probe_i2c_devices()
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

static void read_sensors()
{
    if(file == -1) /* no sensors */
        return;

    if(ioctl(file, I2C_SLAVE, BMA180) < 0) {
        fprintf(stderr, "Failed to aquire bus address\n");
        exit(1);
    }

    int i;

    i2c_read(0x02, 6); /* read data */
    if(buf[0] & 1 && buf[2] & 1 && buf[4] & 1) {
        for(i=0; i<3; i++) {
            dof[i] += (float)(((int16_t)buf[2*i+1]<<8) + (uint8_t)buf[2*i]) / 16384;
            cof[i]++;
        }
    } else
        fprintf(stderr, "accel miss\n");
    
    if(ioctl(file, I2C_SLAVE, ITG3200) < 0) {
        fprintf(stderr, "Failed to aquire bus address\n");
        exit(1);
    }

    i2c_read(0x1a, 1); /* read status byte */
    if(buf[0] & 1) {
        i2c_read(0x1d, 6); /* read data */
        for(i=0; i<3; i++) {
            dof[3+i] += (float)(((int16_t)buf[2*i+0]<<8) + (uint8_t)buf[2*i+1]) / gyro_scale;
            cof[3+i]++;
        }
    } else
        fprintf(stderr, "gyro miss\n");
                    
                    
    if(ioctl(file, I2C_SLAVE, HMC5883) < 0) {
        fprintf(stderr, "Failed to aquire bus address\n");
        exit(1);
    }
                
    /* read data */
    i2c_read(0x09, 1); /* read status byte */
    if(buf[0] & 1) {
        i2c_read(0x03, 6); /* read data */
        for(i=0; i<3; i++) {
            int ind[3] = {0, 4, 2};
            dof[6+i] += (float)((int)((int16_t)buf[ind[i]]<<8) + (uint8_t)buf[ind[i]+1]) / 512;
            cof[6+i]++;
        }
//            i2c_write_byte(0x09, 0x00);
    } else
        fprintf(stderr, "mag miss\n");

    i2c_write_byte(0x02, 0x01); /* request conversion */

    if(ioctl(file, I2C_SLAVE, BMP085) != -1) {
        i2c_read(0xf6, 3);
        dof[9] += (float)(((int32_t)buf[0]<<24) +
                          ((int32_t)buf[1]<<16) +
                          ((int32_t)buf[2])) / (1<<24);  cof[9]++;
        
        i2c_write_byte(0xf4, 0xf4); /* request conversion */
    }
}

void config(int op)
{
    FILE *f = config_open(op, config_name);
    if(!f) {
        fprintf(stderr, "failed to open config: %s\n", config_name);
        return;
    }

    config_string(op, f, "i2c_device", i2c_device, sizeof i2c_device);

    config_float(op, f, "desired_heading", &desired_heading);
    
    /* pid gains, used to compute autopilot command */
    config_float_vector(op, f, "accel_bias", of_bias + 0, 3);
    config_float_vector(op, f, "gyro_bias", of_bias + 3, 3);
    config_float_vector(op, f, "mag_bias", of_bias + 6, 3);

    config_float(op, f, "gyro_bias_lowpass", &gyro_bias_lowpass);
    config_float(op, f, "gyro_scale", &gyro_scale);

    config_float_vector(op, f, "alignment", alignment, 4);

    fclose(f);
}

static void read_config()
{
    config(READ);
}

static void write_config()
{
    config(WRITE);
}

static void compute_initial_orientation()
{
    /* compute initial orientation quaternion
       directly from accels and mags
    */
    
    float q[4], a[3] = {of[0], of[1], of[2]}, m[3] = {of[6], of[7], of[8]};
    float g[3] = {0, 0, 1};
    
    normalize(a);
    normalize(m);
    
    /* remove accerometer read, or "down" component of mag */
    vec2vec2quat(q, a, g);
    rotvecquat(m, q);
    
    m[2] = 0; /* flatten mag */
    normalize(m);
    
    float r[4]; /* rotation for yaw */
    angvec2quat(r, -atan2(m[1], m[0]), g);
    
    /* combine rotations */
    quatmult2(r, q);
    
    q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
}

struct timeval start;
float imu_runtime()
{
    struct timeval current;
    gettimeofday(&current, NULL);
    return current.tv_sec - start.tv_sec + (current.tv_usec - start.tv_usec)/1e6;
}

static void *sensors_loop(void *v)
{
    float freq = 50;
    float l = 0, l2 = 0;
    /* record start time */
    gettimeofday(&start, NULL);

    for(;;) {
        float n = imu_runtime();

        float s = l - n + 1.0/freq;
        if(s<0) {
            fprintf(stderr, "system too slow to maintain inertial sensors filter\n");
        } else {
            struct timespec ts = {0, (s)*1e9};
            nanosleep(&ts, NULL);
        }
        l = imu_runtime();

        float p = 1.0f/filter_sample_count;
        if(n - l2 >= p) {
            l2 +=  p;

            /* average results */
            int i;
            for(i=0; i<SENSOR_COUNT; i++)
                if(cof[i] == 0)
                    goto skip;

            for(i=0; i<SENSOR_COUNT; i++) {
                of[i] = dof[i] / cof[i];
                dof[i] = cof[i] = 0;
            }

            /* long lowpass gyros to cancel bias */
            for(i=3; i<6; i++)
                of_bias[i] = lp_filter(of_bias[i], of[i], gyro_bias_lowpass);

            /* apply bias to all sensors */
            for(i=0; i<SENSOR_COUNT; i++)
                of[i] -= of_bias[i];

            /* on the initial run estimate initial values for orientation */
            static int initial_values = 1;
            if(initial_values) {
                initial_values = 0;
                compute_initial_orientation();
            }

            /* perform sensor fusion */
#if 1
            MadgwickAHRSupdate(of[3], of[4], of[5], of[0], of[1], of[2], of[6], of[7], of[8]);
#else
            MahonyAHRSupdate(of[3], of[4], of[5], of[0], of[1], of[2], of[6], of[7], of[8]);
#endif
            /* calibrate magnetometer biases */
            float magstate[4];
            if(calibrate_mag(of + 6, magstate)) {
                float a[3];
                imu_orientation(a);
                float yaw = a[2];
                for(i=0; i<3; i++)
                    of_bias[MAGX+i] += magstate[i];
                imu_orientation(a);
                imu_yaw_offset(yaw - a[2]); /* cancel yaw shift at current orientation */
            }
            
        }

    skip:
        read_sensors();
    }
}
 
int imu_init()
{
    /* read configuration */
    read_config();
    atexit(write_config);

    /* open i2c bus for inertial sensors */
    if((file = open(i2c_device, O_RDWR)) < 0) {
        fprintf(stderr, "failed to open: '%s'\n", i2c_device);
        return 1;
    }

    init_mag_calibration();
    probe_i2c_devices();

    int i;
    for(i=0; i<SENSOR_COUNT; i++)
        of[i] = dof[i] = cof[i] = 0;

    /* spawn thread to read sensors and apply filter */
    pthread_attr_t attr;
    if(pthread_attr_init(&attr) != 0)
        exit(1);

    pthread_t thread;
    if(pthread_create(&thread, &attr, sensors_loop, NULL) != 0)
        exit(1);

    return 0;
}

/*    s - accel, gyro, mag sensor readings */
void imu_read(float s[9])
{
    int i;
    for(i=0; i<9; i++)
        s[i] = of[i];
}

void imu_yaw_offset(float angle)
{
    float q[4], g[3] = {0, 0, 1};
    angvec2quat(q, deg2rad(angle), g);
    quatmult1(alignment, q);
}

void imu_level()
{
#if 1
    float a[6];
    imu_orientation(a);
    float yaw = a[2];

    alignment[0] = q0;
    alignment[1] = -q1;
    alignment[2] = -q2;
    alignment[3] = -q3;

    imu_orientation(a);
    imu_yaw_offset(yaw - a[2]); /* cancel yaw shift at current orientation */
#else
    /* to ensure the current yaw reading remains the same,
       we compensate pitch then roll and put the result in
       the alignment quaternion */

    /* first compensate pitch */
    float vec1[3] = {aval[0], aval[1], aval[2]};
    normalize(vec1);
 
    float vec2[3] = {0, vec1[1], vec1[2]};
    normalize(vec2);
    
    float q[4];
    vec2vec2quat(q, vec1, vec2);
    quatmult1(q, alignment);

    /* now compensate roll */
    vec1[0] = 0, vec1[1] = 0, vec1[2] = 1;
    vec2vec2quat(q, vec2, vec1);
    quatmult1(q, alignment);
#endif
}

void imu_quaternion(float q[4])
{
    q[0] = q0, q[1] = q1, q[2] = q2, q[3] = q3;
    quatmult1(q, alignment);
 }

/*  OUTPUT: a - pitch roll yaw */
void imu_orientation(float a[3])
{
    float q[4];
    imu_quaternion(q);
    quaternion_pitch_roll_yaw(q, a+0, a+1, a+2);
}

void imu_rate(float a[3])
{
    float q[4];
    imu_quaternion(q);

    /* turn rate in inertial coordinates */
    float r[4], g[3] = {of[GYROX], of[GYROY], of[GYROZ]};
    rotvecquat(g, alignment);
    r[0] = 1 + 0.5f * (-q[1] * g[0] - q[2] * g[1] - q[3] * g[2]);
    r[1] =     0.5f * ( q[0] * g[0] + q[2] * g[2] - q[3] * g[1]);
    r[2] =     0.5f * ( q[0] * g[1] - q[1] * g[2] + q[3] * g[0]);
    r[3] =     0.5f * ( q[0] * g[2] + q[1] * g[1] - q[2] * g[0]);

    quatnormalize(r);
    
    quaternion_pitch_roll_yaw(r, a+0, a+1, a+2);
}
