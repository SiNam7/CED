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
#define _DIST_ALPHA 0.3        // [1628] ema 필터의 측정 보정치

// Filter
#define LENGTH 60
#define k_LENGTH 10

// Servo range
#define _DUTY_MIN 1226     //[0028] servo duty값 최소를 1000으로 고정 222mm
#define _DUTY_NEU 1546        //[3138] servo duty값 중간을 1450으로 고정 190mm
#define _DUTY_MAX 1746     //[3145] servo duty값 최대를 2000으로 고정 167mm

// Servo speed control
#define _SERVO_ANGLE 30        // [3131] servo 각도 설정
#define _SERVO_SPEED 720        // [3141] servo 속도 설정
#define _RAMPUP_TIME 360

// Event periods
#define _INTERVAL_DIST 10 
#define _INTERVAL_SERVO 10 
#define _INTERVAL_SERIAL 100 

// PID parameters
//#define _KP_NEAR 0.99  // 0.99
//#define _KP_FAR 1.02  // 1.02
//#define _KD_NEAR 70.0  // 35.0
//#define _KD_FAR 75.0  // 37.5

#define _KI 0.1
#define _KP 1.4
#define _KD 40.2

#define _ITERM_MAX 30

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
int correction_dist, iter;
float dist_list[LENGTH], sum, alpha;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_max;
int duty_chg_adjust;
int duty_chg_per_interval; 
int duty_target, duty_curr;
int iterm_max;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

float ir_distance_filter();

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO);  // [3228]

//// initialize global variables
  dist_target = _DIST_TARGET;
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;
  iterm_max = _ITERM_MAX;
// move servo to neutral position
 duty_curr = _DUTY_NEU;
 myservo.writeMicroseconds(_DUTY_NEU); // [3228]

// initialize serial port
Serial.begin(57600); //[3128] 시리얼 포트 초기화

// convert angle speed into duty change per interval.

//  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
//  duty_chg_adjust = (float) duty_chg_max * _INTERVAL_SERVO / _RAMPUP_TIME;
//  duty_chg_per_interval = 0; // initial speed is set to 0.
  
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); //[3128]

  dist_ema = 0.0;
  error_prev = 0.0;
}
  
void loop() {

/////////////////////
// Event generator // [3133] 이벤트 실행 간격 구현 
///////////////////// 
{
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;
}
////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
    filtered_dist = ir_distance_filter();
    
  // PID control logic
    error_curr = _DIST_TARGET - filtered_dist; //[0028] 현재 오차 저장
    
//    if (error_curr > 0) {
//      pterm = error_curr * _KP_NEAR;
//      dterm = (error_curr - error_prev) * _KD_NEAR;
//    }
//    else {
//      pterm = error_curr * _KP_FAR;
//      dterm = (error_curr - error_prev) * _KD_FAR;
//    }

    pterm = error_curr * _KP;
    dterm = (error_curr - error_prev) * _KD;
    iterm += _KI * error_curr;
    
    if (iterm > iterm_max) iterm = iterm_max;
    if (iterm < (iterm_max * -1)) iterm = (iterm_max * -1);

    control = pterm + dterm + iterm;
//    control = pterm;
//    control = dterm;

{
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    error_prev = error_curr;
  
    last_sampling_time_dist = millis(); // [3133] 마지막 dist event 처리 시각 기록
}
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

    Serial.print("IR:"); 
    Serial.print(filtered_dist); 
    Serial.print(",T:"); 
    Serial.print(dist_target); 
    Serial.print(",P:"); 
    Serial.print(map(pterm,-1000,1000,510,610)); 
    Serial.print(",D:"); 
    Serial.print(map(dterm,-1000,1000,510,610)); 
    Serial.print(",I:"); 
    Serial.print(map(iterm,-1000,1000,510,610)); 
    Serial.print(",DTT:"); 
    Serial.print(map(duty_target,1000,2000,410,510)); 
    Serial.print(",DTC:"); 
    Serial.print(map(duty_curr,1000,2000,410,510)); 
    Serial.println(",-G:245,+G:265,m:0,M:800");
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

float ir_distance_filter() {
  sum = 0;
  iter = 0;
  while (iter < LENGTH)
  {
    
    dist_raw = ir_distance();
    dist_list[iter] = dist_raw;
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = k_LENGTH; i < LENGTH-k_LENGTH; i++) {
    sum += dist_list[i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH);

  float value = alpha*dist_cali + (1-alpha)*dist_ema;
  dist_ema = value;
  return value;
}
