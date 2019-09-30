/* 
 *  A set of utility functions for interfacing with the MPU9250
 */

void initializeIMUs()//(MPU6050 &imu1, MPU6050 &imu2)
{
      Serial.println(F("Initializing I2C devices..."));
      imu1.initialize();
      imu2.initialize();
}

// verify connection
void testIMUConnections(void) 
{
    Serial.println(F("Testing device 1 connections..."));
    Serial.println(imu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));

    Serial.println(F("Testing device 2 connections..."));
    Serial.println(imu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));
}

/* 
 *  Start the IMU, it's DMP system, and set the enabled features and read speed
 */
uint8_t setupIMU(MPU6050 imu, uint8_t &mpuIntStatus, bool &dmpReady, uint16_t &packetSize, String imuName)
{
    // load and configure the DMP
    Serial.println(F("Initializing DMP on " + imuName));
    devStatus = imu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    imu.setXGyroOffset(51);
    imu.setYGyroOffset(8);
    imu.setZGyroOffset(21);
    imu.setXAccelOffset(1150); 
    imu.setYAccelOffset(-50); 
    imu.setZAccelOffset(1060); 
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        imu.CalibrateAccel(6);
        imu.CalibrateGyro(6);
        Serial.println();
        imu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        imu.setDMPEnabled(true);

        mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP on " + imuName + " ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed on " + imuName + " (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    return devStatus;
}

//Quaternion getQuat(MPU6050 imu)
//{
//    Quaternion q;
//    q.a = imu.calcQuat(imu.qw);
//    q.b = imu.calcQuat(imu.qx);
//    q.c = imu.calcQuat(imu.qy);
//    q.d = imu.calcQuat(imu.qz);
//
//    return q;
//}

void dmp1DataReady() {
    mpu1Interrupt = true;
}

void dmp2DataReady() {
    mpu2Interrupt = true;
}

void printIMUData(MPU6050 &imu1, MPU6050 &imu2)
{          
    imu1.dmpGetQuaternion(&q_arm, fifoBuffer1);
    imu2.dmpGetQuaternion(&q_hand, fifoBuffer2);

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
