#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "servo.h"


float servo_scale = -1000; /* physical configuration */
float min_hardover_period = 2;

float servo_command_hz = 10;

int servo_open(const char *device)
{


    /* configure serial port to drive motor */
    char cmd[256] = "";
    strcat(cmd, "stty -F ");
    strcat(cmd, device);
    strcat(cmd, " 9600 ignbrk -icrnl -opost -onlcr -isig -icanon "
           "-iexten -echo -echoe -echok -echoctl -echoke min 1 time 5");
    system(cmd);

    int servo;
    if((servo = open(device, O_RDWR)) < 0)
        printf("failed to open '%s' to control servo\n", device);
    else if(fcntl(servo, F_SETFL, O_NONBLOCK) == -1) {
        printf("failed to set '%s' non blocking\n", device);
        exit(1);
    }
    return servo;
}

void servo_write(int servo, int command)
{
    char scommand[128] = "";
    float scaled_command = servo_scale*command;
    if(scaled_command > fabsf(servo_scale))
        scaled_command = servo_scale;
    if(scaled_command < -fabsf(servo_scale))
        scaled_command = -servo_scale;
    
    static int lsc;
    int max_command = 2*fabsf(servo_scale) / min_hardover_period * servo_command_hz / 100;
    if(scaled_command - lsc > max_command)
        scaled_command = max_command + lsc;
    if(scaled_command - lsc < -max_command)
        scaled_command = -max_command + lsc;
    lsc = scaled_command;

    sprintf(scommand, "%d\r\n", (int)(scaled_command));
    write(servo, scommand, strlen(scommand));
}

int servo_read(int servo)
{
    char buf[1];
    while((read(servo, buf, 1)) == 1) {
        switch(buf[0]) {
        case 'k': /* accepted command */
            break;
        case 'm': /* monitor reading */
        default: /* unknown */
            break;
        }
    }
    return 0;
}
