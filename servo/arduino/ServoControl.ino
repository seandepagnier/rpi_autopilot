/* Copyright (C) 2013 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* servo driver for arduino.

   basic usage:

   apt-get install arduino-mk
   make
   make upload

   Control motor controllers via pwm pulses
   for rc brushed and brushless controllers, or directly to a servo.

   Prevent stalling by current feedback with shunts on the low side
   (to allow arduino to sample shunts with adc.

   Current feedback is required for speed controlled servos to determine
   when there are stops.

   Current feedback is recommended for position controlled servos
   as it allows them to prevent stalling if an obstruction is present.

In all modes current feedback allows:
  track efficiency over time, monitor power consumption

   The following hardware capabilities are supported (easily extendable)

   2 motors: pins 9, 10
   2 motor current shunts: adc0 to adc1 and adc2 to adc3
   3 buttons: pins 5, 6, 7
   1 buzzer: pins 12 and 13

   this program also supports
   relay control for motor 1 (via pins 2 and 3)

   communication: serial (usb via ftdi)
   These can be tested manually by running minicom

   Control commands: (sent from host to arduino via usb)
   !GETCAP
   !GETVAR <motor#> <variable>
   !SETVAR <motor#> <variable> <value>
   variables:
     MODE -- values: IDLE POSITION SPEED
     CMD  -- command from +-1000
     MAX_CURRENT -- maximum allowed current (in centiamps) from 0 to 1000
                    before stall is detected
     MIN_SPEED -- only in speed mode, from 0 to 1000
            use pwm (motor starts and stops) when speed is lower than
            this value to avoid running the motor very slowly and inefficiently.
   !BUZZ <N> - command buzzer

   Controller replies to !GETCAP
   !CAP <#of Motors> <supported modes> <# of buttons>

   Controller outputs at approx 10hz:
   !STATUS <motor 1> ... <motor 2> <button mask>

   each motor gives: <mode>:<command>:<current>:<flags>

   Motor Modes:
      IDLE - No pwm pulses are generated on this channel
      POSITION - motor position is set directly, and stalling only
                 occurs only because of another fault.
      SPEED - motor speed is set to control a motor via controller.
              when the motor reaches the end of its travel, it will
              stall and automatically be stopped
      FAULT - not user settable, no pwm pulses are generated, see below

   command is the current motor command

   current is given in tenths of amps from 0 to 1000 (0 to 100 amps) with
   a trailing ! if overcurrent is detected

   Motor Flags:
      + - Stall in Positive direction
      - - Stall in Negative direction

  if the + or - flag is set, additional commands in this direction are ignored
  to prevent additional stalling from occuring.

 if stalling (overcurrent) detected, the motor is
   re-commanded backward to eliminate the stall.
   If the current consumption cannot be reduced sufficiently
   in a short period of time, the motor is set as FAULT
*/

#include <Servo.h> 

#include <util/delay.h>

#define MOTOR_COUNT 2

const int stopped = 90; // motor is stopped at servo command of 90 degrees

const int reposition_command = 100; // move at command of 100 to get back in range

int relay_pin0=2, relay_pin1=3;

#define BUTTON1 1
#define BUTTON2 2
#define BUTTON3 4
#define BUTTON_COUNT 3
int button_ground_pin = 4;
int button1 = 5;
int button2 = 6;
int button3 = 7;

int relayduty = 0, currelayduty = 0;
const int minrelayduty = 10;

/* return sign of a number so that x = sign(x)*abs(x) */
int sign(int x) { return (x > 0) ? 1 : (x < 0) ? -1 : 0; }

void relays_forward()
{
    digitalWrite(relay_pin1, LOW);
    digitalWrite(relay_pin0, HIGH);
}

void relays_backward()
{
    digitalWrite(relay_pin0, LOW);
    digitalWrite(relay_pin1, HIGH);
}

void relays_stop()
{
    digitalWrite(relay_pin0, LOW);
    digitalWrite(relay_pin1, LOW);
}

void command_relays(int command)
{
    /* command relays */
    int relayduty=abs(command);
    if(relayduty < minrelayduty)
        relayduty = 0;
    if(relayduty > 100)
        relayduty = 100;

    /* very low speed PWM for the relays,
       with period approximately relay period seconds */
    const int relayperiod = 3;
    if(currelayduty < relayduty*relayperiod) {
        if(command > 0)
            relays_forward();
        else
            relays_backward();
    } else
        relays_stop();
          
    /* increment position in duty cycle */
    currelayduty++;
    if(currelayduty >= 100 * relayperiod)
        currelayduty = 0;
}

struct Motor
{
    Motor() : max_current(1000), min_speed(150) {}

    Servo servo;  // create servo object to control a servo 
    enum Mode {IDLE, POSITION, SPEED, FAULT} mode;
    enum {NONE, POS, NEG, BOTH}; /* stall flags */

    int flags;

    int command;
    int max_current; // maximum allowed current
    int min_speed;

    int over_current_rounds; // number of iterations in a row we are overcurrent
    int max_measured_current; // maximum current read since last status

    void setmode(Mode m, int motor) {
        mode = m;
        switch(mode) {
        case POSITION:
        case SPEED:
            servo.attach(9+motor);
            servo.write(stopped);
            break;
        default:
            mode = IDLE;
            servo.detach();
        }
    }
    
    int duty, curduty;
    void setcommand(int c) {
        command = c;

        if(mode == SPEED) {
            /* for speed mode convert command to speed and duty cycle */
            int speed = abs(command), duty = 100;
            if(speed < min_speed) {
                duty = 100 * speed / min_speed;
                if(duty > 100)
                    duty = 100;
                command = sign(command)*min_speed;
            }

            /* minimize transitions */
            if(servo.read() == stopped) {
                curduty = 0; /* stay stopped */
            } else
                curduty = duty; /* moving */
        }
    }

    void write() {
        if(mode == POSITION) {
            servo.write(stopped + command * 2 / 25);
        } else {
            if(duty<curduty)
                servo.write(stopped + command * 2 / 25);
            else
                servo.write(stopped);
            
            /* increment position in duty cycle */
            if(++curduty >= 100)
                curduty = 0;
        }
    }
    
    void update(int current) {
        if(!servo.attached())
            return;

#if 0
        /* switched direction? some controllers require
           a delay from forward to reverse
           must stop first and wait before reversing */
        if(lastcommand >= 0 && motorcommand < 0) {
            servo.write(stopped);
            delay(500);
        }
#endif

        /* check for overcurrent */
        if(current > max_measured_current)
            max_measured_current = current;
        
        if(current > max_current)
            over_current_rounds++;
        else
            over_current_rounds = 0;
        
        /* overcurrent too long, fault */
        if(over_current_rounds >= 24) {
            fault();
        /* overcurrent, try alternative */
        } else if(over_current_rounds >= 8) {
            if(mode == Motor::POSITION) {
                if(command < 0) {
                    flags = NEG;
                    setcommand(command + 100);
                    if(command >= 0)
                        fault();
                } else {
                    flags = POS;
                    setcommand(command - 100);
                    if(command <= 0)
                        fault();
                }
            } else if(mode == SPEED) {
                servo.write(stopped);
                relays_stop();
                delay(300);
                
                if(command < 0) {
                    flags &= ~NEG;
                    setcommand(200);
                    write();
                    relays_forward();
                } else {
                    flags &= ~POS;
                    setcommand(-200);
                    write();
                    relays_backward();
                }
                
                delay(200);
                setcommand(0);
                write();
                relays_stop();
              }
        } else /* no overcurrent */
            over_current_rounds = 0;
    }

    void fault() {
        servo.detach();
        mode = FAULT;
//        Serial.println("!FAULT %d\n", m);
    }

    void output_state() {
        switch(mode) {
        case IDLE: Serial.print("IDLE"); break;
        case POSITION: Serial.print("POSITION"); break;
        case SPEED: Serial.print("SPEED"); break;
        default: Serial.print("FAULT"); break;
        }
    }
} motors[MOTOR_COUNT];

void delay_us(long us)
{
    long i;
    for(i=0; i<us; i++)
      _delay_us(1);
}

/* sound buzzer for milliseconds duration,
  for duty cycle(0 - 100), period in micro seconds */
void buzz(int ms, int duty, int period)
{
  int buzzer_pin0=12, buzzer_pin1=13;
  
  pinMode(buzzer_pin0, OUTPUT);
  pinMode(buzzer_pin1, OUTPUT);

  digitalWrite(buzzer_pin1, HIGH);
  long cnt = (long)ms*1000L/period;
  long d = (long)period*(long)duty/100;
  for(int i=0; i<cnt; i++) {    
      digitalWrite(buzzer_pin0, HIGH);
      delay_us(d);
      digitalWrite(buzzer_pin0, LOW);
      delay_us(period-d);
  }
}

/* play different tones from 0 <= n <= 8 */
void buzz_cmd(unsigned int n)
{
    n = n%5;
    buzz(300, 5, 2000 + 500*n);
    delay(100);
}

/* play startup notes */
void buzz_startup()
{
  buzz(200, 10, 3200);
  delay(50);
  buzz(200, 10, 2600);
  delay(50);
  buzz(200, 10, 2000);
  delay(50);
}

int read_buttons()
{
   return
     (digitalRead(button1) ? 0 : BUTTON1) |
     (digitalRead(button2) ? 0 : BUTTON2) |
     (digitalRead(button3) ? 0 : BUTTON3);
}

void setup() 
{ 
    // set up Serial library at 9600 bps 
    Serial.begin(9600);
    Serial.println("Servo Control Start");

    // set up pins for relay control
    pinMode(relay_pin0, OUTPUT);
    pinMode(relay_pin1, OUTPUT);

    // set up pins for button inputs
    pinMode(button_ground_pin, OUTPUT);
    digitalWrite(button_ground_pin, LOW);
    digitalWrite(button1, HIGH); /* enable internal pullups */
    digitalWrite(button2, HIGH);
    digitalWrite(button3, HIGH);    

    buzz_startup();
} 

/* read a line from serial port or return NULL */
static char* serial_read_line(void)
{
    static char buffer[32];
    static int bufpos = 0;
    while(Serial.available()) {
        char c = Serial.read();

        if(c == '\n' || c == '\r') {
            buffer[bufpos] = '\0';
            bufpos=0;

            return buffer;
        }

        buffer[bufpos++] = c;
        
        if(bufpos == (sizeof buffer) / (sizeof *buffer)) {
            Serial.println("overflow on input serial buffer");
            bufpos = 0;
        }
    }
    return NULL;
}

int read_current(int servo)
{
    if(servo < 0 || servo > 3)
        return 0;

  int t = 0;
  for(int i=0; i<16; i++) {
    int a = analogRead(A0);
    int b = analogRead(A1+servo);
    t += ((a-b) >> 2);
  }
  return t/16;
}

void output_status()
{
    Serial.print("!STATUS");

    /* mode */
    for(int i=0; i<MOTOR_COUNT;i++) {
        Serial.print(" motor=");
        Serial.print(i);

        Serial.print(" state=");
        motors[i].output_state();

        if(motors[i].mode != Motor::IDLE) {
            /* command */
            Serial.print(" cmd=");
            Serial.print(motors[i].command);
            
            /* current */
            Serial.print(" curr=");
            Serial.print(motors[i].max_measured_current);
        }

        /* flags */
        if(motors[i].flags) {
            Serial.print(" stall=");
            if(motors[i].flags & Motor::POS)
                Serial.print("+");
            if(motors[i].flags & Motor::NEG)
                Serial.print("-");
        }
    }

    Serial.print(" buttons=");
    Serial.println(read_buttons());
}

int cmdcmp(const char *cmd, const char *s, char **endptr)
{
    while(*s==' ') s++;

    int len = strlen(cmd);
    if(endptr) {
        *endptr = (char*)s+len;
        while(**endptr == ' ') *endptr++;
    }
    return !strncmp(cmd, s, len);
}

void loop() 
{
    static int statcntr;
    /* print status every 10 iterations */
    if(++statcntr >= 10) {
        statcntr = 0;
        output_status();
    }

    /* read input */
    char *c = serial_read_line(), *d;
    if(c) {
        if(cmdcmp("!GETCAP", c, 0)) {
          Serial.print("!CAP");

          Serial.print(" MOTORS=");
          Serial.print(MOTOR_COUNT);

          Serial.print(" MODES=IDLE,POSITION,SPEED");

          Serial.print(" BUTTONS=");
          Serial.print(BUTTON_COUNT);

          Serial.println(" BUZZER");
        } else if(cmdcmp("!SETVAR", c, &d)) {
          int m = strtol(d, &c, 10);

          if(cmdcmp("MODE", c, &d)) {
              if(cmdcmp("POSITION", d, 0))
                  motors[m].setmode(Motor::POSITION, m);
              else if(cmdcmp("SPEED", d, 0))
                  motors[m].setmode(Motor::SPEED, m);
              else
                  motors[m].setmode(Motor::IDLE, m);

              if(m == 0)
                  relays_stop();

          } else if(cmdcmp("CMD", c, &d)) {
            int command = strtol(d, NULL, 10);

            if(motors[m].servo.attached())
                if(!motors[m].flags ||
                   (motors[m].flags & Motor::NEG && command >= motors[m].command) ||
                   (motors[m].flags & Motor::POS && command <= motors[m].command)) {
                       motors[m].command = command;
                       motors[m].flags = 0;
                   }
          } else if(cmdcmp("MAX_CURRENT", c, &d)) {
              motors[m].max_current = strtol(d, NULL, 10);
          } else if(cmdcmp("MIN_SPEED", c, &d)) {
              motors[m].min_speed = strtol(d, NULL, 10);
          } else {
              Serial.print("unknown variable: ");
              Serial.println(c);
          }
        } else if(cmdcmp("!BUZZ", c, 0)) {
            int n = strtol(c+5, NULL, 10);
            buzz_cmd(n);
        } else {
            Serial.print("unknown command string: ");
            Serial.println(c);
        }
    }
  
 
  for(int m=0; m<MOTOR_COUNT; m++)
      motors[m].update(read_current(m));
  command_relays(motors[0].command);

  /* run at approximately 100hz */
    delay(10);
}
