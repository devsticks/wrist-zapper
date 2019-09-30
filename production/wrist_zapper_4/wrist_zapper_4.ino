/************************************************************
 * Written by Devin Stickells
 */

//// -- INCLUDES -- ////

#include "I2Cdev.h"                                 // helper functions for IMU interfacing
#include "MPU6050_6Axis_MotionApps_V6_12.h"         // functions specific to the MPU6050
#include "SD.h"                                     // SD card interfacing
#include "helper_3dmath.h"                          // quaternion math
#include "BasicLinearAlgebra.h"                     // matrix math

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"                               // nice GPIO functions
#endif

//// -- CONSTANT DEFINITIONS -- ////

using namespace BLA;                                // BasicLinearAlgebra namespace

#define INTERRUPT_PIN1 32  
#define INTERRUPT_PIN2 14
#define LED_PIN 13 

//// -- GLOBAL VARIABLES -- ////

// AD0 low = 0x68 (default, hand IMU)
// AD0 high = 0x69 (arm IMU)
MPU6050 imu1(0x69), imu2(0x68);

bool blinkState = false;

// MPU control/status vars
bool dmp1Ready = false, dmp2Ready = false;          // flag set true if DMP init was successful
uint8_t mpu1IntStatus, mpu2IntStatus;               // holds actual interrupt status byte from MPU
uint8_t devStatus;                                  // return status after each device operation (0 = success, !0 = error)
uint16_t packet1Size, packet2Size;                                // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1, fifoCount2;                    // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64], fifoBuffer2[64];           // FIFO storage buffer
volatile bool mpu1Interrupt = false;                // indicates whether MPU1 interrupt pin has gone high
volatile bool mpu2Interrupt = false;                // indicates whether MPU2 interrupt pin has gone high

// orientation/motion vars
Quaternion q_arm, q_hand;                           // [w, x, y, z] quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

void setup() {

//// -- SETUP -- ////

    Serial.begin(115200);                             // initialize serial comms

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    setupGPIO();
    initializeIMUs();
    testIMUConnections();

    setupIMU(imu1, mpu1IntStatus, dmp1Ready, packet1Size, "IMU 1");
    setupIMU(imu2, mpu2IntStatus, dmp2Ready, packet2Size, "IMU 2");
    setupSD();

    if (devStatus != 0 || !dmp1Ready || ! dmp2Ready) { /* startupFailed() TODO turn a red light on or something to show there's a problem */ }
    
}

void loop() {
  // Check for new data in the IMU FIFO's
//  if ( imu1.fifoAvailable() || imu2.fifoAvailable() )
//  {
//    imu1.dmpUpdateFifo();
//    imu2.dmpUpdateFifo();
//    //printIMUData();
//    Quaternion q_arm = getQuat(imu1);
//    Quaternion q_hand = getQuat(imu2);
//    float extensionAngle = calcExtensionAngle(q_arm, q_hand);
//    updateShock(extensionAngle, 30, 30, 0, 2);
//  }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpu1Interrupt && fifoCount1 < packet1Size) {
        if (mpu1Interrupt && fifoCount1 < packet1Size) {
          // try to get out of the infinite loop 
          fifoCount1 = imu1.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpu1Interrupt = false;
    mpu1IntStatus = imu1.getIntStatus();

    // get current FIFO count
    fifoCount1 = imu1.getFIFOCount();
    fifoCount2 = imu2.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpu1IntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount1 >= 1024) {
        // reset so we can continue cleanly
        imu1.resetFIFO();
        imu2.resetFIFO();
        fifoCount1 = imu1.getFIFOCount();
        fifoCount2 = imu2.getFIFOCount();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpu1IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount1 < packet1Size) fifoCount1 = imu1.getFIFOCount();

        // read a packet from FIFO
        imu1.getFIFOBytes(fifoBuffer1, packet1Size);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount1 -= packet1Size;
        
        // display quaternion values in easy matrix form: w x y z
        imu1.dmpGetQuaternion(&q_arm, fifoBuffer1);
//        Serial.print("quat1\t");
//        Serial.print(q_arm.w);
//        Serial.print("\t");
//        Serial.print(q_arm.x);
//        Serial.print("\t");
//        Serial.print(q_arm.y);
//        Serial.print("\t");
//        Serial.println(q_arm.z);

        while (fifoCount2 < packet2Size) fifoCount2 = imu2.getFIFOCount();
        
        imu2.getFIFOBytes(fifoBuffer2, packet2Size);
        fifoCount2 -= packet2Size;

        imu2.dmpGetQuaternion(&q_hand, fifoBuffer2);
//        Serial.print("quat2\t");
//        Serial.print(q_hand.w);
//        Serial.print("\t");
//        Serial.print(q_hand.x);
//        Serial.print("\t");
//        Serial.print(q_hand.y);
//        Serial.print("\t");
//        Serial.println(q_hand.z);

        float extensionAngle = calcExtensionAngle(q_arm, q_hand);
        Serial.println(String(extensionAngle));
        updateShock(extensionAngle, 30, 30, 0, 2);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
