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
#include <Quaternion.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

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
  
  if (imu2.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              20) != INV_SUCCESS) // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
  {
    while (1)
    {
      SerialPort.println("Unable to start DMP on MPU-9250 1");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  if (imu1.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              20)  != INV_SUCCESS) // Set DMP FIFO rate to 10 Hz
  {
    while (1)
    {
      SerialPort.println("Unable to start DMP on MPU-9250 2");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
      
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
//    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
//    if (imu1.dmpUpdateFifo() == INV_SUCCESS | imu2.dmpUpdateFifo() == INV_SUCCESS)
//    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      //imu.computeEulerAngles();
      printIMUData();
      //saveToSD();
//    }
  }
}

// convert a percentage of max current to a DAC value
int toDac(int percentage)
{
  float minVoltage = 0.8; // threshold voltage of BJT
  float maxVoltage = 3.1; // max produceable voltage
  int dacMaxVal = 255;

  float outputVoltage = ((percentage * 0.01) * (maxVoltage - minVoltage)) + minVoltage; // calculate the output voltage we want
  int dacVal = round(outputVoltage * (255 / maxVoltage)); // convert required voltage to a DAC value
  
  return dacVal;
}

float norm(BLA::Matrix<3> vec) {
  return sqrt(vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2));
}

// output a specific column from rotation matrix
BLA::Matrix<3> getNormalizedCol(BLA::Matrix<3,3> rotm, char col) {
  BLA::Matrix<3> ret;
  ret = {rotm(0,col), rotm(1,col), rotm(2,col)};
  ret /= norm(ret);
  return ret;
}

BLA::Matrix<3> cross(BLA::Matrix<3> a, BLA::Matrix<3> b)
{
    BLA::Matrix<3> ret;

    ret(0) = a(1) * b(2) - a(2) * b(1);
    ret(1) = a(2) * b(0) - a(0) * b(2);
    ret(2) = a(0) * b(1) - a(1) * b(0);

    return ret;
}

float dot(BLA::Matrix<3> a, BLA::Matrix<3> b)
{
    return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
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
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
    imu1.dmpUpdateFifo();
    imu2.dmpUpdateFifo();
        
    Quaternion q_arm;
    q_arm.a = imu1.calcQuat(imu1.qw);
    q_arm.b = imu1.calcQuat(imu1.qx);
    q_arm.c = imu1.calcQuat(imu1.qy);
    q_arm.d = imu1.calcQuat(imu1.qz);
   
    Quaternion q_hand;
    q_hand.a = imu2.calcQuat(imu2.qw);
    q_hand.b = imu2.calcQuat(imu2.qx);
    q_hand.c = imu2.calcQuat(imu2.qy);
    q_hand.d = imu2.calcQuat(imu2.qz);

    float extensionAngle = calcExtensionAngle(q_arm, q_hand);
    Serial.println(String(extensionAngle));

    int stimRangeStart = 30; // start stimulus at this point
    int stimRange = 30; // range over which intensity varies
    float stimIntensityStart = 0; // percentage intensity at stimRangeStart
    float stimIntensityRange = 2; // increase in percentage intensity from stimRangeStart to end of stimRange

    if (extensionAngle > stimRangeStart + stimRange) // max intensity
    { 
      float intensity = stimIntensityStart + stimIntensityRange;
      int j = toDac(intensity);
      dacWrite(0,j);
      Serial.println("Shocking at max intensity, " + String(intensity) + "%!");
    } 
    else if (extensionAngle > stimRangeStart) 
    {
      float intensity = stimIntensityStart + (extensionAngle - stimRangeStart)/(stimRange / stimIntensityRange);
//      for(int i=0;i<100;i+=10) {
      int j = toDac(intensity);
      dacWrite(0,j);
      Serial.println("Shocking at " + String(intensity) + "%!");
//        delay(2000);            
//      } 
    } else {
        dacWrite(0,0);
        Serial.println("Shock off");
    }
    
//    String imustring = String(q_arm.a) + "," + String(q_arm.b) + "," + String(q_arm.c) + "," + String(q_arm.d) + "," + String(q_hand.a) + "," + String(q_hand.b) + "," + String(q_hand.c) + "," + String(q_hand.d) + "\n";
    
//    appendFile(SD, "/qlog.txt", imustring);
//    appendFile(SD, "/alog.txt", String(extensionAngle) + "\n");
    
//    SerialPort.println(String(q_arm.a) + "," + String(q_arm.b) + "," + String(q_arm.c) + "," + String(q_arm.d) + "," + String(q_hand.a) + "," + String(q_hand.b) + "," + String(q_hand.c) + "," + String(q_hand.d) );
    
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

float calcExtensionAngle(Quaternion q_arm, Quaternion q_hand) 
{

//// -- SETUP -- ////

    // convert arm quaternion to rotation matrix
    BLA::Matrix<3,3> arm_rot = q_arm.to_rotation_matrix();
//    Serial << "ret(2,0): " << arm_rot << '\n'; 

    // pull direction vectors from rotation matrix
    BLA::Matrix<3> arm_x = getNormalizedCol(arm_rot, 0);
    BLA::Matrix<3> arm_y = getNormalizedCol(arm_rot, 1);
    BLA::Matrix<3> arm_z = getNormalizedCol(arm_rot, 2);

    // convert hand quaternion to rotation matrix
    BLA::Matrix<3,3> hand_rot = q_hand.to_rotation_matrix();
//    Serial << "ret(2,0): " << arm_rot << '\n';

    // pull normalized direction vectors from rotation matrix
    BLA::Matrix<3> hand_x = getNormalizedCol(hand_rot, 0);
    BLA::Matrix<3> hand_y = getNormalizedCol(hand_rot, 1);
    BLA::Matrix<3> hand_z = getNormalizedCol(hand_rot, 2);

//// -- FIND PLANES DESCRIBING THE EXTENSION ANGLE -- ////
    
    // find normal of arm "vertical" plane
    BLA::Matrix<3> n_vec_1 = arm_y; // normal of arm plane

    // find normal of hand extension plane
    BLA::Matrix<3> n_vec_2 = cross(hand_x,arm_y); // find normal of hand extension plane
    n_vec_2 /= norm(n_vec_2); // normalise
    
    BLA::Matrix<3> v_intersection = cross(n_vec_1,n_vec_2); // find vector of intersection of planes
    v_intersection /= norm(v_intersection); // normalise

//// -- CALCULATE THE EXTENSION ANGLE -- ////

    float angle = acos(dot(arm_x, v_intersection))*360/(2*PI);

    // find sign
    float sign = 1;
    BLA::Matrix<3> cross_prod = cross(arm_x, v_intersection);
    if (dot(arm_y, cross_prod) > 0) { 
      sign = -1;
    }
    
    return sign * angle;  
}
