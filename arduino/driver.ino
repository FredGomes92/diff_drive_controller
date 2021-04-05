
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


double set_point_pos, set_point_vel = 0, temp = 0.0, vel = 0.0, pos = 0.0;

// PID
// double kp =1, ki =20 , kd =0;
// double input = 0, output = 0, setpoint = 0;

// PID myPID(&input, &output, &setpoint, kp, ki, kd,DIRECT);


void setup() {

  pinMode(M1_OUT, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // PID
  // myPID.SetMode(AUTOMATIC);
  // myPID.SetSampleTime(1);
  // myPID.SetOutputLimits(-255, 255);


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

byte buff[5] = {'p', 1,'v',2,255};

void OnRequestEvent()
{
   uint8_t buff[4];

   //double vel_ = 10;
   //double pos_ = 100;

   int tmp = (unsigned long int)abs((vel*100));

  buff[0] = abs(tmp) >> 8; // MSB
  buff[1] = abs(tmp); // LSB
  buff[2] = abs((int)(pos)) >> 8; // MSB
  buff[3] = abs((int)(pos));

  if (vel < 0)
    buff[0] = buff[0] | 0b10000000;

  // Serial.println("Tmp: " + String(tmp) + "Vel: " + String(vel) + "buff[0]: " + String(buff[0]) + " buff[1]: " + String(buff[1]));

  Wire.write(buff, sizeof(buff));


}

// this function is executed everytime it receives data from the master
// first 2 bytes -> velocity setpoint , last 2 bytes -> position setpoint
void OnReceiveEvent(int c)
{
  uint8_t data[4];

  for (int i = 0; i < 4; i ++)
  {
    data[i] = Wire.read();
    //Serial.println("Data[" + String(i) + "] = " + String(data[i]));
  }

  int sign = (data[0] & 0b10000000) == 0b10000000 ? -1 : 1;

  set_point_vel = (((((data[0] & 0b01111111) << 8) | (data[1]))/100.0))*sign;

  // set_point_pos = (double)(data[2] | data[3] >> 8);

  Serial.println("Set_point_vel: " + String(set_point_vel));

}
// encoder pulses -> 0 to 3300
void PulseCnt()
{

  if (enc_pos > (PPR*10) || enc_pos < -(PPR*10))
      enc_pos = 0;

  if (PINB & 0b0000001) enc_pos --;  // if (digitalRead(ENC_PHB) == HIGH) enc_pos --;
  else enc_pos ++; // if (digitalRead(ENC_PHB) == LOW) enc_pos ++;


  //if (enc_pos > PPR) enc_pos = 0;
  //else if (enc_pos < 0) enc_pos = PPR;

  // Serial.println(enc_pos);

}

void loop() {

  now = millis();
  int timeChange = (now - lastTime);

  pos = ((enc_pos*360) / PPR) % 360;

  if (timeChange >= 100) // calculate vel every 500 ms
  {
    // Serial.println("Pos: " + String(enc_pos) + " Last_pos: " + String(last_enc_pos));
    // Serial.println("Now: " + String(now) + "LastTime : " + String(lastTime));

    temp = ((enc_pos - last_enc_pos)*360.0*1000) / (PPR * (now - lastTime));

    if (abs(enc_pos) > abs(last_enc_pos)) // to save encoderPos at boundary i.e., after max limit it will take the previous value. Then lastPos will be greater than encoderpos
      vel =temp;

    if (enc_pos == last_enc_pos)
      vel = 0;

    lastTime = now;
    last_enc_pos = enc_pos;

    // Serial.println("Vel: " + String(vel));
    // Serial.println("EncPos: " + String(enc_pos) + "LastEncPos: " + String(last_enc_pos));

    Serial.println(pos);

  }
  SetPwm(set_point_vel);
}
