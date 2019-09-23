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
#define MPU9250_ADDRESS_1 0x68   // Device address when ADO = 0
#define MPU9250_ADDRESS_2 0x69   // Device address when ADO = 1

MPU9250_DMP imu1;
MPU9250_DMP imu2;

void setup() 
{
  int MPU1 = 0x68;
  int MPU2 = 0x69;
  
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu1.begin(0x69) != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250 1");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  if (imu2.begin(0x68) != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250 2");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  Serial.println("1 " + String(imu1.getAddr()) + ", 2 " + String(imu2.getAddr()));
  
  imu1.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  imu2.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
              
  //setup SD card
  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu1.fifoAvailable() || imu2.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu1.dmpUpdateFifo() == INV_SUCCESS)
    {
      imu2.dmpUpdateFifo();
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      //imu.computeEulerAngles();
      printIMUData();
      //saveToSD();
    }
  }
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
    imu1.dmpUpdateFifo();
    float q1w = imu1.calcQuat(imu1.qw);
    float q1x = imu1.calcQuat(imu1.qx);
    float q1y = imu1.calcQuat(imu1.qy);
    float q1z = imu1.calcQuat(imu1.qz);

    imu2.dmpUpdateFifo();
    float q2w = imu2.calcQuat(imu2.qw);
    float q2x = imu2.calcQuat(imu2.qx);
    float q2y = imu2.calcQuat(imu2.qy);
    float q2z = imu2.calcQuat(imu2.qz);
    
    SerialPort.println(String(q1w) + "," + String(q1x) + "," + String(q1y) + "," + String(q1z) + "," + String(q2w) + "," + String(q2x) + "," + String(q2y) + "," + String(q2z) );
    
//  SerialPort.println("Q: " + String(q0, 4) + ", " +
//                    String(q1, 4) + ", " + String(q2, 4) + 
//                    ", " + String(q3, 4));
//  SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
//            + String(imu.pitch) + ", " + String(imu.yaw));
//  SerialPort.println("Time: " + String(imu.time) + " ms");
//  SerialPort.println();
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

void saveToSD(void)
{
    float q1w = imu1.calcQuat(imu1.qw);
    float q1x = imu1.calcQuat(imu1.qx);
    float q1y = imu1.calcQuat(imu1.qy);
    float q1z = imu1.calcQuat(imu1.qz);

    float q2w = imu2.calcQuat(imu2.qw);
    float q2x = imu2.calcQuat(imu2.qx);
    float q2y = imu2.calcQuat(imu2.qy);
    float q2z = imu2.calcQuat(imu2.qz);
    
    //String headerString = "ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,ax2,ay2,az2,gx2,gy2,gz2,mx2,my2,mz2";
    String IMUString = String(q1w) + "," + String(q1x) + "," + String(q1y) + "," + String(q1z) + "," + String(q2w) + "," + String(q2x) + "," + String(q2y) + "," + String(q2z) + "\n";
    //const char * IMU1Char = (const char *)IMU1String;
    //appendFile(SD, "/log.txt", headerString + "\n");
    appendFile(SD, "/qlog.txt", IMUString);
}
