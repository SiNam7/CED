#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define _DUTY_MIN 1226 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 1776 // servo full counterclockwise position (180 degree)

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define _SERVO_SPEED 60 // servo speed limit (unit: degree/second)
#define INTERVAL 10  // servo update interval

// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr;

const float coE[] = {0.0000446, -0.0111011, 1.6833671, 23.8816948};

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);// initialize serial port
  
  Serial.begin(57600);

//  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;

  // initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = coE[0] * pow(raw_dist, 3) + coE[1] * pow(raw_dist, 2) + coE[2] * raw_dist + coE[3];
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);

//  if(millis() < last_sampling_time + INTERVAL) return;
    if (dist_cali < 255) duty_target = _DUTY_MAX;

    else if (dist_cali > 255) duty_target = _DUTY_MIN;
  Serial.print(",duty_target:");
  Serial.println(duty_target);
//
//// adjust duty_curr toward duty_target by duty_chg_per_interval
//  if(duty_target > duty_curr) {
//    duty_curr += duty_chg_per_interval;
//    if(duty_curr > duty_target) duty_curr = duty_target;
//  }
//  else {
//    duty_curr -= duty_chg_per_interval;
//    if(duty_curr < duty_target) duty_curr = duty_target;
//  }

  
  
  myservo.writeMicroseconds(duty_target);
  delay(20);
}
