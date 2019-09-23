/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/

// from examples/MPU9250_DMP_Quaternion

#include <SparkFunMPU9250-DMP.h>
#include "SD.h"
//#include "MPU9250.h"

#ifdef defined(SAMD)
 #define SerialPort SerialUSB
#else
  #define SerialPort Serial
#endif

// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO_1 0
#define ADO_2 1
// Define I2C addresses of the two MPU9250
#define MPU1 0x68   // Device address when ADO = 0
#define MPU2 0x69   // Device address when ADO = 1

MPU9250_DMP imu;

void setup() 
{
//  int MPU1 = 0x68;
//  int MPU2 = 0x69;
  
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin(0x68) != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250 1");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  // Call imu.begin() to verify communication and initialize
  if (imu.begin(0x69) != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250 1");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
              
  //setup SD card
  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();
}

void loop() 
{
  bool readReady = false;
  bool updated = false;

  imu.setAddr(MPU1);
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  { readReady = true; }
  else
  { 
    imu.setAddr(MPU2);
    if ( imu.fifoAvailable() )
    { readReady = true; }
  }

  if (readReady) {
    printIMUData();
  //saveToSD();
    readReady = false;
  }
}

// from microsd_test - consider moving to external library
void appendFile(fs::FS &fs, const char * path, /*const char * */ String message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void printIMUData(void)
{      
    float q1w;
    float q1x;
    float q1y;
    float q1z;
    float q2w;
    float q2x;
    float q2y;
    float q2z;
    
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
//    imu.setAddr(MPU1);
//    if (imu.fifoAvailable()) {
//      imu.dmpUpdateFifo();
//      Serial.println(imu.getAddr());
//      q1w = imu.calcQuat(imu.qw);
//      q1x = imu.calcQuat(imu.qx);
//      q1y = imu.calcQuat(imu.qy);
//      q1z = imu.calcQuat(imu.qz);
////      SerialPort.println(String(q1w) + "," + String(q1x) + "," + String(q1y) + "," + String(q1z));
//    } else {
//      q1w = 0;
//      q1x = 0;
//      q1y = 0;
//      q1z = 0;
//    }
//
//    imu.setAddr(MPU2);
//    if (imu.fifoAvailable()) {
//      Serial.println(imu.getAddr());
//      imu.dmpUpdateFifo();
//      q2w = imu.calcQuat(imu.qw);
//      q2x = imu.calcQuat(imu.qx);
//      q2y = imu.calcQuat(imu.qy);
//      q2z = imu.calcQuat(imu.qz);
////      SerialPort.println(String(q2w) + "," + String(q2x) + "," + String(q2y) + "," + String(q2z));
//    } else {
//      q2w = 0;
//      q2x = 0;
//      q2y = 0;
//      q2z = 0;
//    }

    imu.setAddr(MPU1);
    if (imu.fifoAvailable()) {
      imu.dmpUpdateFifo();}
      
    q1w = imu.calcQuat(imu.qw);
    q1x = imu.calcQuat(imu.qx);
    q1y = imu.calcQuat(imu.qy);
    q1z = imu.calcQuat(imu.qz);

    imu.setAddr(MPU2);
    if (imu.fifoAvailable()) {
      imu.dmpUpdateFifo();
    }
    
    q2w = imu.calcQuat(imu.qw);
    q2x = imu.calcQuat(imu.qx);
    q2y = imu.calcQuat(imu.qy);
    q2z = imu.calcQuat(imu.qz);
    
    String IMUString = String(q1w) + "," + String(q1x) + "," + String(q1y) + "," + String(q1z) + "," + String(q2w) + "," + String(q2x) + "," + String(q2y) + "," + String(q2z) + "\n";
    SerialPort.print(IMUString);
    appendFile(SD, "/qlog2.txt", IMUString);
    
//  SerialPort.println("Q: " + String(q0, 4) + ", " +
//                    String(q1, 4) + ", " + String(q2, 4) + 
//                    ", " + String(q3, 4));
//  SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
//            + String(imu.pitch) + ", " + String(imu.yaw));
//  SerialPort.println("Time: " + String(imu.time) + " ms");
//  SerialPort.println();
}

void saveToSD(void)
{
    imu.setAddr(MPU1);
    float q1w = imu.calcQuat(imu.qw);
    float q1x = imu.calcQuat(imu.qx);
    float q1y = imu.calcQuat(imu.qy);
    float q1z = imu.calcQuat(imu.qz);

    imu.setAddr(MPU2);
    float q2w = imu.calcQuat(imu.qw);
    float q2x = imu.calcQuat(imu.qx);
    float q2y = imu.calcQuat(imu.qy);
    float q2z = imu.calcQuat(imu.qz);
    
    //String headerString = "ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,ax2,ay2,az2,gx2,gy2,gz2,mx2,my2,mz2";
    String IMUString = String(q1w) + "," + String(q1x) + "," + String(q1y) + "," + String(q1z) + "," + String(q2w) + "," + String(q2x) + "," + String(q2y) + "," + String(q2z) + "\n";
    //const char * IMU1Char = (const char *)IMU1String;
    //appendFile(SD, "/log.txt", headerString + "\n");
    appendFile(SD, "/qlog.txt", IMUString);
}
