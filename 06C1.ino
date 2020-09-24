// 5V - R - (+ LED -) - pin7, 전원과 보드 GND 연결

#define PIN_LED 7

int p, cycle;

void set_period(int period) {
  p = period;
}

void set_duty(int duty) {
  int dly = p / 100 * duty;
  digitalWrite(PIN_LED, 0);
  delayMicroseconds(dly);
  
  digitalWrite(PIN_LED, 1);
  delayMicroseconds(p - dly);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  set_period(10000);
}

void loop() {
  int cycle = 5000 / p;
  if (p == 10000) cycle = 10000 / p;

  for (int j = 0; j < 100; j++) {
    for (int i = 0; i < cycle; i++)
    set_duty(j);
  }

  for (int j = 100; j > 0; j--) {
    for (int i = 0; i < cycle; i++)
    set_duty(j);
  }
}
