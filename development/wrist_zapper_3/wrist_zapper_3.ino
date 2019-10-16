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

//// -- GLOBAL VARIABLES -- ////

MPU9250_DMP imu1;
MPU9250_DMP imu2;

const unsigned char ARM_IMU_ADDRESS = 0x68;        // Device address when ADO = 0
const unsigned char HAND_IMU_ADDRESS = 0x69;       // Device address when ADO = 1

void setup() {

//// -- SETUP -- ////

  Serial.begin(115200);

  setupIMU(imu1, ARM_IMU_ADDRESS, "IMU 1");
  setupIMU(imu2, HAND_IMU_ADDRESS, "IMU 2");

  setupSD();
}

void loop() {
  // Check for new data in the IMU FIFO's
  if ( imu1.fifoAvailable() || imu2.fifoAvailable() )
  {
    imu1.dmpUpdateFifo();
    imu2.dmpUpdateFifo();
    //printIMUData();
    Quaternion q_arm = getQuat(imu1);
    Quaternion q_hand = getQuat(imu2);
    float extensionAngle = calcExtensionAngle(q_arm, q_hand);
    updateShock(extensionAngle, 30, 30, 0, 2);
  }

}
