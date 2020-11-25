#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  //[1692] LED 9번핀에 연결
#define PIN_SERVO 10 // [3228] 서보10핀에 연결
#define PIN_IR A0  // [3133] 적외선 센서 signal -> A0핀

// Framework setting
#define _DIST_TARGET 255    //[0028] 목표 위치가 25.5cm임을 선언
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.0        // [1628] ema 필터의 측정 보정치

// Servo range
#define _DUTY_MIN 1226     //[0028] servo duty값 최소를 1000으로 고정 
#define _DUTY_NEU 1476        //[3138] servo duty값 중간을 1450으로 고정
#define _DUTY_MAX 1776     //[3145] servo duty값 최대를 2000으로 고정

// Servo speed control
#define _SERVO_ANGLE 30        // [3131] servo 각도 설정
#define _SERVO_SPEED 360        // [3141] servo 속도 설정

// Event periods
#define _INTERVAL_DIST 10 
#define _INTERVAL_SERVO 10 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 1.3    

//////////////////////
// global variables //
//////////////////////
//float dist_min, dist_max, event_dist, dist_raw, filtered_dist; // [3228]
const float coE[] = {0.0000446, -0.0111011, 1.6833671, 23.8816948};
//const float coE[] = {0.0000237, -0.0024448, 0.6002129, 61.3219813};
// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, filtered_dist;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO);  // [3228]

//// initialize global variables
//  dist_min = _DIST_MIN;
//  dist_max = _DIST_MAX;   // [3129]

// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU); // [3228]

// initialize serial port
Serial.begin(57600); //[3128] 시리얼 포트 초기화

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); //[3128]
}
  
void loop() {
/////////////////////
// Event generator // [3133] 이벤트 실행 간격 구현 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      event_dist = false; // [3133]
  // get a distance reading from the distance sensor
      dist_raw = ir_distance();
//      filtered_dist = ir_distance_filtered(dist_raw);    //[0028] 적외선 센서 필터링 값 저장
      filtered_dist = dist_raw;
    
  // PID control logic
    error_curr = _DIST_TARGET - filtered_dist; //[0028] 현재 오차 저장
    pterm = error_curr * _KP;    //[0028] kp * 오차
    control = pterm;    //[0028] 제어량 계산

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
  
    last_sampling_time_dist = millis(); // [3133] 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false; // [3133]
    
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);

    last_sampling_time_servo = millis(); // [3133] 마지막 servo event 처리 시각 기록

  }
  
  if(event_serial) {
    event_serial = false; // [3133]
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis(); // [3133] 마지막 serial event 처리 시각 기록

  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return coE[0] * pow(val, 3) + coE[1] * pow(val, 2) + coE[2] * val + coE[3];
  // [3129] 적외선 센서 거리 측정
}
