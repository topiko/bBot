// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// stepper library
#include <AccelStepper.h>


// use servos:
#include <Servo.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;


/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// command given by rpi
unsigned char command[3] = {0, 0, 0};
int pitchInt;
volatile boolean inSer = false;
unsigned long time_read_imu = 0;
unsigned long prev_imu_read_time = 0;
unsigned long dt = 0;
int comIdx = 0;
int n1 = 0;
int n2 = 0;
int n3 = 0;
boolean usb = false; //true;

// Stepper motor variables:
// leg pins:

#define dirPin_Lleg 50
#define stepPin_Lleg 48
#define dirPin_Rleg 42
#define stepPin_Rleg 40
// hand pins:
#define dirPin_Rhand 34
#define stepPin_Rhand 32
#define dirPin_Lhand 26
#define stepPin_Lhand 24

// This tels that we have a driver for the stepper:
#define motorInterfaceType 1

// Max speed and acc:
#define maxStepperSpeed 3069 // 2048
#define maxStepperAcc 65535

AccelStepper lLeg = AccelStepper(motorInterfaceType, stepPin_Lleg, dirPin_Lleg);
AccelStepper rLeg = AccelStepper(motorInterfaceType, stepPin_Rleg, dirPin_Rleg);
AccelStepper lHand = AccelStepper(motorInterfaceType, stepPin_Lhand, dirPin_Lhand);
AccelStepper rHand = AccelStepper(motorInterfaceType, stepPin_Rhand, dirPin_Rhand);

// stepper setup pins:
#define enablePin 52
#define stepModeCOM0Hand 38
#define stepModeCOM1Hand 36
#define stepModeCOM0Leg 44
#define stepModeCOM1Leg 46
#define enablePinHands 28
#define enablePinLegs 30


int lLeg_speed = 0;
int rLeg_speed = 0;
int lHand_speed = 0;
int rHand_speed = 0;

//MultiStepper steppers; // upt to 10 steppers we can use MultiStepper

// Head tilt servo:
# define servoPin 12
Servo headServo;  // create servo object to control a servo

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

// RPI serial interrupt:
// =========================
void serialEvent3() {
  inSer = true;
}
// =========================

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  // Setup stepper pins:
  pinMode(stepModeCOM0Leg, OUTPUT);
  pinMode(stepModeCOM1Leg, OUTPUT);
  pinMode(stepModeCOM0Hand, OUTPUT);
  pinMode(stepModeCOM1Hand, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(enablePinHands, OUTPUT);
  pinMode(enablePinLegs, OUTPUT);

  // pulls drivers from sleep mode:
  digitalWrite(enablePin, HIGH);

  // disables steppers
  digitalWrite(enablePinLegs, HIGH);
  digitalWrite(enablePinHands, HIGH);

  // full step
  digitalWrite(stepModeCOM0Leg, HIGH);
  digitalWrite(stepModeCOM1Leg, LOW);

  // 1/8 microstepping
  digitalWrite(stepModeCOM0Hand, HIGH);
  digitalWrite(stepModeCOM1Hand, HIGH);


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  if (usb) Serial.begin(115200);


  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  if (usb) Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  if (usb) Serial.println(F("Testing device connections..."));
  if (usb) Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  else mpu.testConnection();

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  if (usb) Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  /*
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  */

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    if (usb) Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    if (usb) Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

    attachInterrupt(23, dmpDataReady, RISING);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    if (usb) Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    if (usb) Serial.print(F("DMP Initialization failed (code "));
    if (usb) Serial.print(devStatus);
    if (usb) Serial.println(F(")"));
  }

  // Stepper configurations:
  // speeds:
  lLeg.setMaxSpeed(maxStepperSpeed);
  rLeg.setMaxSpeed(maxStepperSpeed);
  lHand.setMaxSpeed(maxStepperSpeed);
  rHand.setMaxSpeed(maxStepperSpeed);
  // acccels:
  lLeg.setAcceleration(maxStepperAcc);
  rLeg.setAcceleration(maxStepperAcc);
  lHand.setAcceleration(maxStepperAcc);
  rHand.setAcceleration(maxStepperAcc);

  /*
    steppers.addStepper(lLeg);
    steppers.addStepper(rLeg);
    steppers.addStepper(lHand);
    steppers.addStepper(rHand);
  */

  //servo:
  headServo.attach(servoPin);
  headServo.write(128);

  // Signal if dmp initialize was succesfull by nodding head:
  if (!dmpReady) {
    for (int i = 0; i < 200; i++) {
      if (i % 2 == 0) headServo.write(0);
      else if (i % 2 == 1) headServo.write(128);
      delay(10);
    }
  }
  else {
    for (int i = 0; i < 10; i++) {
      if (i % 2 == 0) headServo.write(0);
      else if (i % 2 == 1) headServo.write(128);
      delay(1000);
    }
  }

  // Wait for raspberry to start
  delay(1000 * 30);

  // RPI serial
  Serial3.begin(115200);

  // Nod head to say hello
  for (int i = 0; i < 10; i++) {
    if (i % 2 == 0) headServo.write(0);
    else if (i % 2 == 1) headServo.write(128);
    delay(100);
  }

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

    // run the motors
    lLeg.setSpeed(lLeg_speed);
    rLeg.setSpeed(rLeg_speed);
    lHand.setSpeed(lHand_speed);
    rHand.setSpeed(rHand_speed);

    lHand.run();
    rHand.run();
    lLeg.run();
    rLeg.run();

    if (inSer) {
      inSer = false;

      // only compute if requested:
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      comIdx = 0;
      while (comIdx < 3) {
        while (Serial3.available() > 0) {
          command[comIdx] = Serial3.read();
          comIdx++;
        }
      }

      n1 = command[0] >> 6;
      n2 = (((command[0] & 63) << 5) | (command[1] >> 3)) - 1024;
      n3 = ((command[1] & 7) << 8 | command[2]) - 1024;

      pitchInt = (int)(20860 * ypr[1]); // * 180/M_PI)
      dt = time_read_imu - prev_imu_read_time;

      Serial3.write(pitchInt >> 8);
      Serial3.write(pitchInt & 0xff);
      Serial3.write(dt >> 8);
      Serial3.write(dt & 0xff);

      //Serial3.write(time_read_imu >> 24);
      //Serial3.write(time_read_imu >> 16);
      //Serial3.write(time_read_imu >> 8);
      //Serial3.write(time_read_imu & 0xff);

      prev_imu_read_time = time_read_imu;

      if (usb) {
        Serial.print("Pitch int: ");
        Serial.print(pitchInt);
        Serial.print("\t");
        Serial.print("FiFO count ");
        Serial.print(mpu.getFIFOCount());
        Serial.print("\t");
        Serial.print("pitch\t");
        Serial.print(ypr[1] * 180 / M_PI); //ypr[1] * 180/M_PI);
        Serial.print("\tcommand\t");
        Serial.print(n1);
        Serial.print("\t");
        Serial.print(n2);
        Serial.print("\t");
        Serial.println(n3);
      }

      if (n1 == 0) {
        lLeg_speed = 3 * n2;
        rLeg_speed = 3 * n3;
        lLeg.moveTo(lLeg.currentPosition() + 3 * n2);
        rLeg.moveTo(rLeg.currentPosition() + 3 * n3);
      }
      else if (n1 == 1) {
        lHand_speed = n2;
        lHand.moveTo(lHand.currentPosition() + n3);
      }
      else if (n1 == 2) {
        rHand_speed = n2;
        rHand.moveTo(rHand.currentPosition() + n3);
      }
      else if (n1 == 3) {
        if (n2 == 0) {
          // disable motors
          digitalWrite(enablePinLegs, HIGH);
          digitalWrite(enablePinHands, HIGH);
        }
        else if (n2 == 1) {
          // enable Legs
          digitalWrite(enablePinLegs, LOW);
        }
        else if (n2 == 2) {
          // enable hannds
          digitalWrite(enablePinHands, LOW);
        }
        else if (n2 == 3) {
          // note 0 <= n2 < 256:
          headServo.write(n3);
        }
      }
    }
  }


  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    if (usb) {
      Serial.println(F("FIFO overflow!"));
    }
    else {
      Serial3.write("0");
      Serial3.write("1");
    }
    //Serial3.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    time_read_imu = micros();

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }

}
