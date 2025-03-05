#include <PID_v1.h>

#define PIN_FEEDBACK A0
#define PIN_MOSFET 11  // Change to use timer 2 for PWM
#define PIN_MOSFET1 10

double Input, Output;
double Setpoint;
double Kp = 0.01, Ki = 30, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double R1 = 47000.0;
double R2 = 4700.0;

void setup() {
  Serial.begin(9600);
  Input = 0;
  

  // Set up PWM on pin 11 (timer 2, channel A) with 20 kHz
  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(PIN_MOSFET1, OUTPUT);

  TCCR2A = (1 << WGM20) | (1 << WGM21);  // Fast PWM mode
  TCCR2B = (1 << CS21);  // No prescaler, maximum frequency
  OCR2A = 128;  // Set the duty cycle (0 to 255), e.g., 50%
  TCCR2A |= (1 << COM2A1);

  int dutyCycle = 50;  // Desired duty cycle in percentage
  int maxDutyValue = 255;  // The maximum value for OCR2A
  int ocr2aValue = (dutyCycle * maxDutyValue) / 100;
  OCR2A = ocr2aValue;
  TCCR2B = (1 << CS20);  // Set prescaler to 8, resulting in 20 kHz PWM frequency

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  int val = map(analogRead(A1),0,1023,7,30);
  Setpoint = val;
  float val1 = 0;
  for(int i =0; i<= 20; i++){
    val1 += analogRead(PIN_FEEDBACK);
    delay(10);
  }
  Input = val1/20;
  Input = Input / 1023.0;
  Input = Input * 55.0;

  myPID.Compute();

  float dutyCycle = Output/255.0 * 150.0;
  analogWrite(PIN_MOSFET, dutyCycle);
  Serial.print("SET:");
  Serial.print(",");
  Serial.print(Setpoint);
  Serial.print("VOL:");
  Serial.print(",");
  Serial.print(Input);
  Serial.print(",");
  Serial.print("PWM:");
  Serial.print(",");
  Serial.print(dutyCycle);
  Serial.println();
  delay(100);
}
