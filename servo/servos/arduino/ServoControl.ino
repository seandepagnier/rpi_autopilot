/* simple auto pilot control driver
   take input commands from -1000 to 1000 over serial
   and command rc brushless motor controller to speed
   and duty cycle */

#include <Servo.h> 

#include <util/delay.h>
 
Servo servo1, servo2;  // create servo object to control a servo 

const int analogInPin = A2;  // Analog input pin that the potentiometer is attached to
const int minspeed = 15; // don't go slower than 15% of full speed
const int bounds = 80; // keep within 80% of range

const int stopped = 90; // motor is stopped at servo command of 90 degrees

const int reposition_command = 100; // move at command of 100 to get back in range

int pin0=2, pin1=3;

#define BUTTON1 1
#define BUTTON2 2
#define BUTTON3 4
int button_ground_pin = 4;
int button1 = 5;
int button2 = 6;
int button3 = 7;

int command = 0;
int duty = 0, curduty = 0;

int relayduty = 0, currelayduty = 0;
int overcurrentrounds = 0;
const int minrelayduty = 10;

static bool outofbounds = false;
int lastcommand = 0;

int accept_negative_commands = 1, accept_positive_commands = 1;

int autopilot_engauged = 1;

static int button_hold_time, lastbuttons;
const int button_hold_timeout = 30;

int read_buttons()
{
   return
     (digitalRead(button1) ? 0 : BUTTON1) |
     (digitalRead(button2) ? 0 : BUTTON2) |
     (digitalRead(button3) ? 0 : BUTTON3);
}

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
   
void buzz_warning()
{
  buzz(500, 5, 4000);
  delay(500);
  buzz(500, 5, 4000);
  delay(500);
  buzz(500, 5, 4000);
  delay(2000);
}

void buzz_disengauge()
{
  buzz(300, 10, 2000);
  delay(100);
  buzz(300, 10, 2500);
  delay(100);
  buzz(300, 10, 3000);
  delay(100);
}

void buzz_engauge()
{
  buzz(300, 10, 3000);
  delay(100);
  buzz(300, 10, 2500);
  delay(100);
  buzz(300, 10, 2000);
  delay(100);
}

void buzz_invalid()
{
  buzz(300, 10, 4000);
  delay(100);
  buzz(300, 10, 2000);
  delay(100);
}

void buzz_startup()
{
  buzz(200, 10, 3200);
  delay(50);
  buzz(200, 10, 2900);
  delay(50);
  buzz(200, 10, 2600);
  delay(50);
  buzz(200, 10, 2300);
  delay(50);
  buzz(200, 10, 2000);
  delay(50);
}


void buzz_port(int l)
{
  buzz(300, 10, 3500);
  if(l) {
      delay(50);
      buzz(300, 10, 3500);
  }
}

void buzz_starboard(int l)
{
  buzz(300, 10, 3000);
  if(l) {
      delay(50);
      buzz(300, 10, 3000);
  }
}


void forward()
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin0, HIGH);
}

void backward()
{
  digitalWrite(pin0, LOW);
  digitalWrite(pin1, HIGH);
}

void stop()
{
  digitalWrite(pin0, LOW);
  digitalWrite(pin1, LOW);
}

void setup() 
{ 
  Serial.begin(9600);           // set up Serial library at 9600 bps
//  Serial.println("Auto Pilot Control");
  servo1.attach(9);  // attaches the servo on pin 9 to the servo object 
  servo2.attach(10);  // attaches the servo on pin 9 to the servo object 

  servo1.write(stopped);
  servo2.write(stopped);

  stop();
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);

  pinMode(button_ground_pin, OUTPUT);
  digitalWrite(button_ground_pin, LOW);
  digitalWrite(button1, HIGH);
  digitalWrite(button2, HIGH);
  digitalWrite(button3, HIGH);    

  buzz_startup();
} 

int sign(int x)
{
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
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

int read_current(void)
{
  int t = 0;
  for(int i=0; i<16; i++) {
    int a = analogRead(A0);
    int b = analogRead(A1);
    t += ((a-b) >> 2);
  }
  return t/16;
}

void loop() 
{
  const char *c = serial_read_line();
  char *endptr;
  if(c) { 
      long ms = strtol(c+1, &endptr, 10);

      switch(c[0]) {
      case 'm': /* monitor sensors */
          Serial.print("current ");
          Serial.println(current);
          break;
      case 's': /* set current limit */
      case 'c':
          if(c == endptr)
              Serial.println("i"); /* invalid */
          else if(ms < -1000 || ms > 1000)
              Serial.println("f"); /* range */
          else {
              if(c[0] == 's') {
                  maxcurrent = ms;
                  Serial.println("k");
              } else if(c[0] == 'c') {
                  if(!autopilot_engauged)
                      Serial.println("n");
                  else {
                      if(!outofbounds && (accept_negative_commands || ms > 0)
                         && (accept_positive_commands || ms < 0)) {
                          command=ms;
                          accept_negative_commands = 1;
                          accept_positive_commands = 1;
                      }
                      Serial.println("k");
                  }
              }
          }
      }
  }
 
  int buttons = read_buttons();
  if(buttons == lastbuttons) {
    if(autopilot_engauged) {      
      if(buttons == BUTTON1) {
        if(button_hold_time == 0) {
          buzz_port(1);
          Serial.println("P");
          button_hold_time = button_hold_timeout;
        } else
          button_hold_time--;
      }
      if(buttons == BUTTON3) {
        if(button_hold_time == 0) {
          buzz_starboard(1);
          Serial.println("S");
          button_hold_time = button_hold_timeout;
        } else
          button_hold_time--;
      }
    }
  } else {
    button_hold_time = -1;

    if(lastbuttons == BUTTON2 && buttons == 0) {
      if(autopilot_engauged) {
        buzz_disengauge();
        autopilot_engauged = 0;
      } else {
        buzz_engauge();
        autopilot_engauged = 1;
      }
    }

    if(autopilot_engauged) {
      if(buttons == BUTTON1 && lastbuttons == 0) {
        Serial.println("p");
        buzz_port(0);
        button_hold_time = button_hold_timeout;
      }

      if(buttons == BUTTON3 && lastbuttons == 0) {
        Serial.println("s");
        buzz_starboard(0);
        button_hold_time = button_hold_timeout;
      }
    }

    lastbuttons = buttons;
  }

  if(!autopilot_engauged) {
    if(buttons == BUTTON1 && accept_positive_commands) {
      command = 100;
      accept_negative_commands = 1;
    } else
    if(buttons == BUTTON3 && accept_negative_commands) {
      command = -100;
      accept_positive_commands = 1;
    } else
      command = 0;
  }
  
#if 0
  Serial.print("cmd ");
  Serial.print(command);
  Serial.print(" button ");
  Serial.println(buttons);
#endif

  /* check for overcurrent */
  current = read_current();
  if(current > max_current)
      max_current = current;
  if(abs(current) > max_current)
      overcurrentrounds++;
  else
      overcurrentrounds = 0;

  if(overcurrentrounds >=8 ) {
      stop();
      delay(300);
      if(command < 0) {
          accept_negative_commands = 0;
          forward();
      }
      else {
          accept_positive_commands = 0;
          backward();
      }
      delay(200);
      command = 0;
      stop();
      
      Serial.println("e");
    buzz_warning();
  }

  /* command relays */
  int relayduty=abs(command);
  if(relayduty < minrelayduty)
    relayduty = 0;
  if(relayduty > 100)
    relayduty = 100;
    
  if(currelayduty < relayduty*3) {
    if(command > 0)
      forward();
    else
      backward();
  } else
      stop();

  /* increment position in duty cycle */
  currelayduty++;
  if(currelayduty >= 300)
      currelayduty = 0;

  /* convert command to speed and duty cycle */
  int speed = abs(command) / 10, duty = 100;
  if(speed < minspeed) {
      duty = abs(command) * 100 / minspeed;
      speed = minspeed;
  }
  
  /* controller goes from +- 50 from centered value */
  int motorcommand = sign(command) * speed / 2;
  
#if 0
  /* switched direction? must stop first and wait before reversing */
  if(lastcommand >= 0 && motorcommand < 0) {
      servo1.write(stopped);
      stop();
      delay(500);
  }
#endif

  lastcommand = motorcommand;
  
  int servocommand = command * 2 / 25;
  servo1.write(stopped + servocommand);

  /* increment position in duty cycle */
  curduty++;
  if(curduty >= 100)
    curduty = 0;

  /* delay and for really low duties delay longer, otherwise
     nominally run at 100hz */
  if(duty && duty < 20)
    delay(200 / duty);
  else
    delay(10);
}
