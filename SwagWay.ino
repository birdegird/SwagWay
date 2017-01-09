#include <Thread.h>
#include <ThreadController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


ThreadController mainThread = ThreadController();

Thread* outputThread = new Thread();
Thread* sensorThread = new Thread();
Thread* motorThread = new Thread();


/* L298N pins, ENA/ENB must be hardware PWM outputs */
#define ENA 9
#define IN1 4
#define IN2 5

#define ENB 10
#define IN3 6
#define IN4 7


MPU6050 mpu;


#define INTERRUPT_PIN 2


volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];


Quaternion rotation;
VectorFloat gravity;
float euler[3];
float ypr[3];


int16_t motorRegister[3];
float target[3];
bool calibrated = false;


void dmpDataReady()
{
  mpuInterrupt = true;
}


void setup()
{
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);

  if (initMPU())
  {
    sensorThread->onRun(sensorCallback);
    sensorThread->setInterval(5);
    
    mainThread.add(sensorThread);
  }

  if (initL298N())
  {
    motorThread->onRun(motorCallback);
    motorThread->setInterval(10);

    mainThread.add(motorThread);
  }

  Serial.println(F("Calibrating . . ."));

  outputThread->onRun(outputCallback);
  outputThread->setInterval(1000);
  
  mainThread.add(outputThread);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}


void loop()
{
  mainThread.run();
}


void outputCallback()
{
  if (!calibrated) return;
  
  Serial.println();
  
  /*Serial.print(F("Euler\t"));
  Serial.print(euler[0] * 180 / M_PI);
  Serial.print(F("\t"));
  Serial.print(euler[1] * 180 / M_PI);
  Serial.print(F("\t"));
  Serial.println(euler[2] * 180 / M_PI);*/

  Serial.print(F("YPR\t"));
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print(F("\t"));
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print(F("\t"));
  Serial.println(ypr[2] * 180 / M_PI);

  switch (motorRegister[0])
  {
    case 1:
      Serial.print(F("Forward: "));
      Serial.println(motorRegister[1]);
      
      break;
    case 0:
      Serial.print(F("Center: "));

      if (motorRegister[1] == 0)
        Serial.println(F("IDLE"));
      else if (motorRegister[1] == 255)
        Serial.println(F("BRAKING"));
      
      break;
    case -1:
      Serial.print(F("Backward: "));
      Serial.println(motorRegister[1]);
      
      break;
  }

  Serial.print(F("Target: "));
  Serial.print(target[0] * 180 / M_PI);
  Serial.print(F("\t"));
  Serial.print(target[1] * 180 / M_PI);
  Serial.print(F("\t"));
  Serial.println(target[2] * 180 / M_PI);

  //Serial.println(motorRegister[0]);
  //Serial.println(motorRegister[1]);
}


void sensorCallback()
{
  if (!mpuInterrupt) return;

  mpuInterrupt = false;
  
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    mpu.resetFIFO();
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&rotation, fifoBuffer);
    mpu.dmpGetEuler(euler, &rotation);
    mpu.dmpGetGravity(&gravity, &rotation);
    mpu.dmpGetYawPitchRoll(ypr, &rotation, &gravity);
  }
}


void motorCallback()
{
  if (millis() / 1000 < 10 && !calibrated)
  {
    target[0] = (target[0] + ypr[0]) / 2;
    target[1] = (target[1] + ypr[1]) / 2;
    target[2] = (target[2] + ypr[2]) / 2;
  }
  else if (!calibrated)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    
    calibrated = true;
  }
  else
  {
    double targetAngle = (ypr[2] - target[2]) * 255 / M_PI;

    if (abs(targetAngle) > 0 && abs(targetAngle) < 1)
    {
      /* Ugly hack to "force" an autonomous erection */
      target[0] = (target[0] + ypr[0]) / 3;
      target[1] = (target[1] + ypr[1]) / 3;
      target[2] = (target[2] + ypr[2]) / 3;
    }
    
    motorControl(targetAngle);
  }
}


bool initMPU()
{
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  if (mpu.testConnection())
  {
    if (mpu.dmpInitialize() == 0)
    {
      mpu.setXAccelOffset(150);
      mpu.setYAccelOffset(-1185);
      mpu.setZAccelOffset(1138);
      
      mpu.setXGyroOffset(0);
      mpu.setYGyroOffset(0);
      mpu.setZGyroOffset(0);

      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();

      return true;
    }
  }

  return false;
}


bool initL298N()
{
  motorRegister[0] = 0;
  motorRegister[1] = 0;
  motorRegister[2] = 0;
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  return true;
}


void motorControl(double out)
{
  if (out >= 15)
    motorRegister[0] = 1;
  else if (out > -15 && out < 15)
    motorRegister[0] = 0;
  else
    motorRegister[0] = -1;

  int velocity = abs(out);
  
  motorRegister[1] = (velocity > 5) ? (velocity < 15) ? 255 : velocity * M_PI : 0;

  switch (motorRegister[0])
  {
    case 1:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      break;
    case 0:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);

      if (motorRegister[1] == 255 && motorRegister[2] < 20)
      {
        motorRegister[2]++;

        if (motorRegister[2] >= 20)
          motorRegister[2] = 0;
        else if (motorRegister[2] >= 10)
          motorRegister[1] = 0;
      }
      
      break;
    case -1:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);

      break;
  }

  if (motorRegister[1] > 255)
    motorRegister[1] = 255;

  analogWrite(ENA, motorRegister[1]);
  analogWrite(ENB, motorRegister[1]);
}

