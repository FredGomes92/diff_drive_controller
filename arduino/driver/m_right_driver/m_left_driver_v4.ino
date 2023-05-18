#include <Wire.h>
#include <PID_v2.h>


// M1 motor
#define M1_OUT 10
#define IN1 7  // to define CW and CCW directions
#define IN2 8

// Encoder
#define ENC_PHA 3
#define ENC_PHB 2

#define PPR 700  // pulses per rotation

volatile long enc_pos, last_enc_pos = 0, last_encpos = 0;
unsigned int long lastTime, now;
double out = 0.0, outout = 0.0;


// PID
double kp = 0.3, ki = 0.8, kd = 0.07;
double input = 0, output = 0, setpoint = 0.0, setpoint_ = 0.0;

PID myPID(&input, &output, &setpoint_, kp, ki, kd, DIRECT);

float pwm = 0.0;

int8_t pos;

void setup() {

  pinMode(M1_OUT, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(100);


  // I2C
  Wire.begin(10);                  // configure slave in the address #11
  Wire.onRequest(OnRequestEvent);  // data request to slave
  Wire.onReceive(OnReceiveEvent);  // data received by slave

  // Encoder A
  pinMode(ENC_PHA, INPUT_PULLUP);
  pinMode(ENC_PHB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_PHA), PulseCnt, RISING);

  TCCR1B = TCCR1B & 0b11111000 | 1;

  Serial.begin(9600);

  Serial.println("I2C save -> ready");
}


void SetPwm(float out) {
  if (setpoint > 0) {  // CW
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (setpoint < 0) {  // CCW
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {  // stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(M1_OUT, abs(out));
}


void OnRequestEvent() {

  pos = ((360.0 * (enc_pos - last_encpos)) / PPR);  // change in position -> º ... -1 because the direction is the opposite of the left motor

  last_encpos = enc_pos;

  Wire.write(pos);
}

void OnReceiveEvent(int c) {

  uint8_t buff[2];

  for (int i = 0; i < 2; ++i) {
    buff[i] = Wire.read();
  }

  // uint8_t a,b;
  // a = Wire.read();
  // b = Wire.read();
  // setpoint= (double)((b<<8)|a);

  setpoint = (double)((buff[0]) | (buff[1] << 8));
  setpoint_ = abs(setpoint);

  //Serial.println((uint8_t)buff[0]);
  //Serial.println((uint8_t)buff[1]);
  //Serial.println("SetPoint(received): " + String(setpoint));
}
void PulseCnt() {
  if (PIND & (1 << PD2))enc_pos++;  // if (digitalRead(ENC_PHB) == HIGH) enc_pos --;
  else enc_pos--;                   // if (digitalRead(ENC_PHB) == LOW) enc_pos ++;
   
}

void loop() {

  now = millis();
  int td = (now - lastTime);

  if (td >= 100) {

    input = (360.0 * 1000 * abs((enc_pos - last_enc_pos))) / (PPR * (now - lastTime));

    lastTime = now;
    last_enc_pos = enc_pos;

    myPID.Compute();  // calculate new output

    SetPwm(output);   // drive L298N H-Bridge module
    delay(10);
  }
  

  //Serial.println(String(DIR));
  //delay(100);
  //Serial.println("encpos: " + String(enc_pos) + "lastencpos: " + String(last_enc_pos));

  //Serial.println("den: " + String((PPR * (td))));
  //delay(500);
  //Serial.println("nom: " + String((360.0 * 1000 * (enc_pos - last_enc_pos))));
  Serial.println("setpoint: " + String(setpoint) + " input: " + String(input) + " output: " + String(output) + " pos: " + String(pos) + "encpos: " + String(enc_pos) + "lastencpos: " + String(last_enc_pos) + " now: " + String(now) + " lasttime: " + String(lastTime) + " TT: " +String((td)));
}
