/* servo driver for arduino.
   Control motor controllers via pwm pulses
   for rc brushed and brushless controllers, or directly to a servo
   prevent stalling via current feedback.

   with following hardware capabilities:

   4 motors: pins 8, 9, 10, 11
   4 motor current shunts: adc0 to low side and
                        adc1, adc2, adc3, and adc4
                        to high side
   3 buttons: pins 5, 6, 7
   1 buzzer: pins 12 and 13

   this program also supports
   relay control for motor 1 (via pins 2 and 3)

   communication: serial (usb via ftdi)

   Control commands: (sent from host to arduino via usb)
   !GETCAP
   !SETMODE <motor#> <max current> IDLE, POSITION, SPEED 
   !CMD <motor#> <+-1000> - position/speed command
   !BUZZ <N> - command buzzer

   Controller replies:
   !CAP <#of Motors> <supported modes> <# of buttons> <#of beeps>
   !OVERCURRENT <motor#>
   !FAULT <motor#>

   Controller outputs at approx 10hz:
   !STATUS <motor 1> ... <motor 2> <button mask>

   each motor gives: mode:cmd:current:flags

   Motor Modes:
      IDLE - No pwm pulses are generated on this channel
      POSITION - motor position is set directly, and stalling only
                 occurs only because of another fault.
      SPEED - motor speed is set to control a motor via controller.
              when the motor reaches the end of its travel, it will
              stall and automatically be stopped
      FAULT - not user settable, no pwm pulses are generated, see below

   cmd is the current motor command

   current is given in tenths of amps from 0 to 1000 (0 to 100 amps)

   Motor Flags:
      N - None
      + - Stall in Positive direction
      - - Stall in Negative direction
      B - Stall Both

  if the + or - flag is set, additional commands in this direction are ignored
  to prevent additional stalling from occuring

 if stalling (overcurrent) detected, the motor is
   re-commanded backward to eliminate the stall.
   If the current consumption cannot be reduced sufficiently
   in a short period of time, the motor is set as FAULT
*/

#include <Servo.h> 

#include <util/delay.h>

#define MOTOR_COUNT 4

struct Motor
{
    Servo servo;  // create servo object to control a servo 
    enum {IDLE, POSITION, SPEED, FAULT} mode;
    enum {NONE, POS, NEG, BOTH} flags;
    
    int duty, curduty;
    void setcommand(int c) {
        command = c;

        if(mode == SPEED) {
            /* for speed mode convert command to speed and duty cycle */
            int speed = abs(command), duty = 100;
            if(speed < minspeed) {
                duty = 100 * speed / minspeed;
                if(duty > 100)
                    duty = 100;
                command = sign(command)*minspeed;
            }

            /* minimize transitions */
            if(servo.read() == stopped) {
                curduty = 0; /* stay stopped */
            } else
                curduty = duty; /* moving */
        }
    }

    void update()
    {
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

    void fault() {
        motors[motor].detach();
        motors[motor].mode = FAULT;
        Serial.println("!FAULT %d\n", m);
    }

    int command;
    int max_current; // maximum allowed current
    int over_current_rounds; // number of iterations in a row we are overcurrent
    int max_measured_current; // maximum current read since last status
} motors[MOTOR_COUNT];

const int minspeed = 150; // don't go slower than 15% of full speed
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
    int n = n%5;
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

void relay_forward()
{
    digitalWrite(relay_pin1, LOW);
    digitalWrite(relay_pin0, HIGH);
}

void relay_backward()
{
    digitalWrite(relay_pin0, LOW);
    digitalWrite(relay_pin1, HIGH);
}

void relay_stop()
{
    digitalWrite(relay_pin0, LOW);
    digitalWrite(pin1, LOW);
}

void command_relays()
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
    static int bufpos;
    while(Serial.available()) {
        char c = Serial.read();
        if(c == '\n') {
            buffer[bufpos] = '\0';
            bufpos=0;
            return buffer;
        }
        buffer[bufpos++] = c;
        
        if(bufpos == (sizeof buffer) / (sizeof *buffer)) {
            Serial.println("o");
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
    for(int i=0; i<NUM_MOTORS;i++) {
        switch(motor[i].mode) {
        case IDLE: Serial.print("IDLE"); break;
        case POSITION: Serial.print("POSITION"); break;
        case SPEED: Serial.print("SPEED"); break;
        default: Serial.print("FAULT"); break;
        }
        Serial.print(" ");

        Serial.print(motors[m].max_measured_current);
        motors[m].max_measured_current = 0;
        Serial.print(" ");
        
        switch(motor[i].flags) {
        case NONE: Serial.print("N"); break;
        case POS:  Serial.print("+"); break;
        case NEG:  Serial.print("-"); break;
        case BOTH: Serial.print("B"); break;
    }
    Serial.println(read_buttons());
}

int cmdcmp(const char *cmd, const char *s)
{
    return !strncmp(cmd, s, strlen(cmd));
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
    const char *c = serial_read_line();
    char *endptr;
    if(c) {
        if(cmdcmp("!GETCAP", c)) {
          Serial.print("!CAP ");
          Serial.print(MOTOR_COUNT);
          Serial.print(" IDLE:POSITION:SPEED ");
          Serial.print(BUTTON_COUNT);
          Serial.println(" 5");
        } else
        if(cmdcmp("!SETMODE", c) {
          int motor = strtol(c+8, &endptr, 10);
          int maxcurrent = strtol(endptr, &endptr, 10);
          motors[motor].maxcurrent = maxcurrent;

          while(*endptr == ' ') endptr++;
          if(cmdcmp("IDLE", endptr)) {
              motors[motor].detach();
              motors[motor].mode = IDLE:
          } else if(cmdcmp("POSITION", endptr)) {
              servos[motor].attach(9+motor);  // attaches the servo on pin 9 to the servo object 
              servos[motor].write(stopped);
              stop(motor);
          }
          servos[i].write(stopped);
          stop();
          
          servos[i].attach(9+i);  // attaches the servo on pin 9 to the servo object 
          servos[i].write(stopped);
          stop();
       } else if(cmdcmp("!CMD", c)) {
            int motor = strtol(c+4, &endptr, 10);
            int command = strtol(endptr, NULL, 10);

            if(motors[m].servo.attached())
                if((motor[m].flags != NEG && ms >= motor[m].command) ||
                   (motor[m].flags != POS && ms <= motor[m].command)) {
                    motor[m].command=ms;
                    motor[m].flags = ANY;
                }

        } else
        if(cmdcmp("!BUZZ", c)) {
            int n = strtol(c+5, NULL, 10);
            buzz_cmd(n);
        } else
            Serial.print("unknown command string: %s\n", c);

        long ms = strtol(c+1, &endptr, 10);
      }
  }
 
  for(int m=0; m<NUM_MOTORS; m++) {
      if(motors[m].servo.attached()) {
          /* check for overcurrent */
          int current = read_current(m);
          if(current > motors[m].max_measured_current)
              motors[m].max_measured_current = current;

          if(current > motors[m].max_current)
              motors[m].over_current_rounds++;
          else
              motors[m].over_current_rounds = 0;

          /* overcurrent too long, fault */
          if(motors[m].over_current_rounds >= 24) {
              motors[m].fault();
          /* overcurrent, try alternative */
          } else if(motors[m].over_current_rounds >= 8) {
              if(motors[m].mode == POSITION) {
                  if(motors[m].command < 0) {
                      motors[m].flags &= ~NEG;
                      motors[m].setcommand(motors[m].command + 100);
                      if(motors[m].command >= 0)
                          motors[m].fault();
                  } else {
                      motors[m].flags &= ~POS;
                      motors[m].setcommand(motors[m].command - 100);
                      if(motors[m].command <= 0)
                          motors[m].fault();
                  }
              } else if(motors[m].mode == SPEED) {
                  motors[i].servo.write(stopped);
                  relays_stop();
                  delay(300);
                  
                  if(motors[m].command < 0) {
                      motors[m].flags &= ~NEG;
                      motors[m].setcommand(200)
                      relays_forward();
                  } else {
                      motors[m].flags &= ~POS;
                      motors[m].command = -200;
                      motors[m].write();
                      relays_backward();
                  }

                  delay(200);
                  motors[m].command = 0;
                  motors[m].command = 0;
                  motors[m].write();
                  relays_stop();
      
                  Serial.println("!OVERCURRENT %d\n", m);
              }
          } else /* no overcurrent */
              motors[m].over_current_rounds = 0;
      }
  
      if(m==0)
          command_relays();
      motor[m].update();

#if 0
        /* switched direction? some controllers require
           a delay from forward to reverse
           must stop first and wait before reversing */
        if(lastcommand >= 0 && motorcommand < 0) {
            motors[m]servo.write(stopped);
            stop();
            delay(500);
        }
#endif
   }
  /* run at approximately 100hz */
    delay(10);
}
