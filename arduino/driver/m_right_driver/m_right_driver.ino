
#include <Wire.h>
#include <PID_v1.h>


// M1 motor
#define M1_OUT 10
#define IN1 7 // to define CW and CCW directions
#define IN2 8

// Encoder
#define ENC_PHA 3
#define ENC_PHB 4

#define PPR 330 // pulses per rotation

volatile long enc_pos, last_enc_pos = 0;
unsigned int long lastTime, now;


// PID
double kp =1, ki =20 , kd =0;
double input = 0, output = 0, setpoint = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd,DIRECT);

float pwm = 0.0;


void setup() {

  pinMode(M1_OUT, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);


  // I2C
  Wire.begin(10); // configure slave in the address #10
  Wire.onRequest(OnRequestEvent); // data request to slave
  Wire.onReceive(OnReceiveEvent); // data received by slave

  // Encoder A
  pinMode(ENC_PHA, INPUT_PULLUP);
  pinMode(ENC_PHB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_PHA), PulseCnt, RISING);

  TCCR1B = TCCR1B & 0b11111000 | 1;

  Serial.begin(9600);

  Serial.println("I2C save -> ready");

}

void SetPwm(float out)
{
  if (out > 0) //CW
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else //CCW
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(M1_OUT, abs(out));


}

void OnRequestEvent()
{
  int8_t pos;

  pos = (360.0*(enc_pos - last_enc_pos)) / 330.0; // change in position -> º

  // Serial.println("enc_pos: " + String(enc_pos) + " last_enc_pos: " + String(last_enc_pos));
  // Serial.println("Pos: " + String(pos));

  last_enc_pos = enc_pos;

  Wire.write(pos);


}

void OnReceiveEvent(int c)
{
  uint8_t buff[2];

  for (int i = 0; i < 2; ++i)
  {
    buff[i] =Wire.read();
  }

  setpoint = (double)((buff[0])| (buff[1] >> 8));

  Serial.println((uint8_t)buff[0]);
  Serial.println((uint8_t)buff[1]);
  Serial.println("SetPoint: " + String(setpoint));


}
void PulseCnt()
{
   if (enc_pos >= (PPR*10) || enc_pos <= -(PPR*10))
      enc_pos = 0;

  if (PINB & 0b0000001) enc_pos --;  // if (digitalRead(ENC_PHB) == HIGH) enc_pos --;
  else enc_pos ++; // if (digitalRead(ENC_PHB) == LOW) enc_pos ++;


  // Serial.println(enc_pos);
}

float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void loop() {

    now = millis();
    int td = (now - lastTime);

    if (td >= 100)
    {
        input = (360.0*1000*(enc_pos-last_enc_pos))/(330.0*(now - lastTime));

        lastTime = now;
        last_enc_pos = enc_pos;

        // Serial.println("Vel: " + String(input));
    }

    myPID.Compute();                                    // calculate new output
    SetPwm(output);                                     // drive L298N H-Bridge module
    delay(10);
    Serial.println("setpoint: " + String(setpoint) + " pwm: " + String(output));
    SetPwm(pwm);
 }
  
