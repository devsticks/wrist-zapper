/************************************************************
 * Written by Devin Stickells
 */

//// -- INCLUDES -- ////

#include <SparkFunMPU9250-DMP.h>    // to access DMP features of MPU9250
#include "SD.h"                     // for SD card interfacing
#include <Quaternion.h>             // for doing quaternion math
#include <BasicLinearAlgebra.h>     // for doing matrix math

//// -- CONSTANT DEFINITIONS -- ////

using namespace BLA;                // BasicLinearAlgebra namespace

#ifdef defined(SAMD)
 #define SerialPort SerialUSB
#else
  #define SerialPort Serial
#endif

#define ARM_IMU_ADDRESS 0x68        // Device address when ADO = 0
#define HAND_IMU_ADDRESS 0x69       // Device address when ADO = 1

//// -- GLOBAL VARIABLES -- ////

MPU9250_DMP imu1;
MPU9250_DMP imu2;

void setup() {
  // put your setup code here, to run once:
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
