/* 
 *  A set of utility functions for interfacing with the MPU9250
 */

/* 
 *  Start the IMU, it's DMP system, and set the enabled features and read speed
 */
void setupIMU(MPU9250_DMP imu, unsigned char address, String imuName)
{
  // Call imu.begin() to verify communication and initialize
  if (imu.begin(address) != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with " + imuName);
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  if (imu2.dmpBegin(DMP_FEATURE_6X_LP_QUAT |    // Enable 6-axis quat
                    DMP_FEATURE_GYRO_CAL,       // Use gyro calibration
                    20) != INV_SUCCESS)         // Set DMP FIFO rate to 20 Hz
  {
    while (1)
    {
      SerialPort.println("Unable to start DMP on " + imuName);
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
}

Quaternion getQuat(MPU9250_DMP imu)
{
    Quaternion q;
    q.a = imu.calcQuat(imu.qw);
    q.b = imu.calcQuat(imu.qx);
    q.c = imu.calcQuat(imu.qy);
    q.d = imu.calcQuat(imu.qz);

    return q;
}

void printIMUData(void)
{          
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
