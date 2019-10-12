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
//    imu.setXGyroOffset(51);
//    imu.setYGyroOffset(8);
//    imu.setZGyroOffset(21);
//    imu.setXAccelOffset(1150); 
//    imu.setYAccelOffset(-50); 
//    imu.setZAccelOffset(1060); 
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
//        imu.CalibrateAccel(6);
        imu.CalibrateGyro(6);
        Serial.println();
        imu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        imu.setDMPEnabled(true);

        Serial.println(String(imu.getXGyroOffset()));
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

//void printIMUData(MPU6050 &imu1, MPU6050 &imu2)
//{          
//    imu1.dmpGetQuaternion(&q_arm, fifoBuffer1);
//    imu2.dmpGetQuaternion(&q_hand, fifoBuffer2);
//
//    float extensionAngle = calcExtensionAngle(q_arm, q_hand);
//    Serial.println(String(extensionAngle));
//
//    int stimRangeStart = 30; // start stimulus at this point
//    int stimRange = 30; // range over which intensity varies
//    float stimIntensityStart = 0; // percentage intensity at stimRangeStart
//    float stimIntensityRange = 2; // increase in percentage intensity from stimRangeStart to end of stimRange
//
//    // don't want the wearer getting inadvertantly shocked mid-calibration
//    if (calibrated && (extensionAngle > stimRangeStart + stimRange)) // max intensity
//    { 
//      float intensity = stimIntensityStart + stimIntensityRange;
//      int j = toDac(intensity);
//      dacWrite(0,j);
//      Serial.println("Shocking at max intensity, " + String(intensity) + "%!");
//    } 
//    else if (calibrated && (extensionAngle > stimRangeStart)) 
//    {
//      float intensity = stimIntensityStart + (extensionAngle - stimRangeStart)/(stimRange / stimIntensityRange);
////      for(int i=0;i<100;i+=10) {
//      int j = toDac(intensity);
//      dacWrite(0,j);
//      Serial.println("Shocking at " + String(intensity) + "%!");
////        delay(2000);            
////      } 
//    } else {
//        dacWrite(0,0);
//        Serial.println("Shock off");
//    }
    
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
//}

void calibrate() 
{
  calibrated = false;
  calibrating = true;
  updateShock(0);                                                         // stop the wearer from being shocked while calibrating
  int calibCount = 0;                                                     // count how many good readings we have
  Quaternion q_calib_sum_vert = Quaternion(1,0,0,0);                      // the Quaternion sum over which to average the vertical calibration
  Quaternion q_calib_sum_horz = Quaternion(1,0,0,0);                      // the Quaternion sum over which to average the horizontal calibration
  float angle_sum = 0;                                                    // the angle sum, display and recording purposes

  q_calib = Quaternion(1,0,0,0);                                          // reset calibration quaternion so we don't carry range set with previous calibration into next one
  extensionAngle = calcExtensionAngle(q_arm, q_hand); 
  Serial.println("Calib extension angle is " + String(extensionAngle));

  lcdPrint(0, "Arm and hand");
  lcdPrint(1, "straight down.");
  
  Blynk.setProperty(BLYNK_START_CALIB_PIN, "offLabel", "Running");  

  while (calibCount < 60)                                                // we want 150 readings over which to average
  {
    while (/*(abs(100*q_hand.y - 71) > 5) ||*/ (abs(100*q_arm.y - 71) > 5) || (abs(100*(q_arm.x + q_arm.z)) > 10))   // wait until hand is roughly flat relative to arm and arm vertically downwards
    { 
            
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
      } else if (mpu1IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
      {
          updateIMUs();                                                     // update quaternion readings from IMU FIFOs
          extensionAngle = calcExtensionAngle(q_arm, q_hand);               // update angle 
      }
  
      Serial.println("Hand isn't in calibration range");
      Serial.println("Calib extension angle is " + String(extensionAngle));
    
      digitalWrite(EXTERNAL_RED_LED_PIN, 1);                                // turn on red LED to say we're not happy
      digitalWrite(EXTERNAL_GREEN_LED_PIN, 0);                              // turn off green LED to say we're not happy
      Blynk.setProperty(BLYNK_CALIB_LED_PIN, "color", "#D3435C");           // change Blynk app's calibrating indicator LED to red

      calibCount = 0;                                                       // restart reading counter
    }

    // happy with range, calculate...

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
    } else if (mpu1IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
    {
        updateIMUs();                                                     // update quaternion readings from IMU FIFOs
        extensionAngle = calcExtensionAngle(q_arm, q_hand);               // update angle 
    }

    Serial.println("Calibrating...");
  
    digitalWrite(EXTERNAL_RED_LED_PIN, 0);                                // turn off red LED - we're happy
    digitalWrite(EXTERNAL_GREEN_LED_PIN, 1);                              // turn on green LED - we're happy
    Blynk.setProperty(BLYNK_CALIB_LED_PIN, "color", "#43D35C");           // change Blynk app's calibrating indicator LED to green        

    q_calib_sum_vert = q_calib_sum_vert.getSum(q_arm.getProduct(q_hand.getConjugate()));  // calculate rotation of hand frame to arm frame and add to average
    angle_sum += extensionAngle;
    calibCount += 1;
  }

  lcdPrint(0, "Place hand flat,");
  lcdPrint(1, "elbow on table.");

  delay(2000);

  calibCount = 0;

  VectorFloat gravity;        // [x, y, z]            gravity vector
  float arm_ypr[3];           // [yaw, pitch, roll] 
  imu1.dmpGetGravity(&gravity, &q_arm);
  imu1.dmpGetYawPitchRoll(arm_ypr, &q_arm, &gravity);
  float arm_pitch = arm_ypr[1] * 180/M_PI;
  float arm_roll = arm_ypr[2] * 180/M_PI;

  while (calibCount < 100)                                                // we want 100 readings over which to average
  {   
    while (abs(extensionAngle) > 15 || abs(arm_pitch) > 15 || abs(arm_roll) > 15)    // wait until hand is roughly flat relative to arm and arm flat relative to ground
    { 
            
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
      } else if (mpu1IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
      {
          updateIMUs();                                                     // update quaternion readings from IMU FIFOs
          extensionAngle = calcExtensionAngle(q_arm, q_hand);               // update angle 
      }

      imu1.dmpGetGravity(&gravity, &q_arm);
      imu1.dmpGetYawPitchRoll(arm_ypr, &q_arm, &gravity);
      arm_pitch = arm_ypr[1] * 180/M_PI;
      arm_roll = arm_ypr[2] * 180/M_PI;
  
      Serial.println("Hand isn't in calibration range");
    
      digitalWrite(EXTERNAL_RED_LED_PIN, 1);                                // turn on red LED to say we're not happy
      digitalWrite(EXTERNAL_GREEN_LED_PIN, 0);                              // turn off green LED to say we're not happy
      Blynk.setProperty(BLYNK_CALIB_LED_PIN, "color", "#D3435C");           // change Blynk app's calibrating indicator LED to red

      calibCount = 0;                                                       // restart reading counter
    }

// happy with range, calculate...

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
    } else if (mpu1IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) 
    {
        updateIMUs();                                                     // update quaternion readings from IMU FIFOs
        extensionAngle = calcExtensionAngle(q_arm, q_hand);               // update angle 
    }

    Serial.println("Calibrating...");
  
    digitalWrite(EXTERNAL_RED_LED_PIN, 0);                                // turn off red LED - we're happy
    digitalWrite(EXTERNAL_GREEN_LED_PIN, 1);                              // turn on green LED - we're happy
    Blynk.setProperty(BLYNK_CALIB_LED_PIN, "color", "#43D35C");           // change Blynk app's calibrating indicator LED to green        

    q_calib_sum_horz = q_calib_sum_horz.getSum(q_arm.getProduct(q_hand.getConjugate()));  // calculate rotation of hand frame to arm frame and add to average
    angle_sum += extensionAngle;
    calibCount += 1;
  }

  q_calib = (q_calib_sum_horz.getProduct(0.9)).getSum(q_calib_sum_horz.getProduct(0.1));

  q_calib = q_calib.getProduct(0.00625);
  q_calib.normalize();
  Serial.println("w:" + String(q_calib.w) + " x:" + String(q_calib.x) + " y:" + String(q_calib.y) + " z:" + String(q_calib.z));
  extensionAngle = angle_sum / 150;

  Serial.println("Calibrated baseline extension angle is " + String(extensionAngle));
  
  appendFile(SD, "/event_log.txt", getDateTimeString() + ", " + "Calibrated: Baseline extension angle is " + String(extensionAngle) + "\n");
        
  digitalWrite(EXTERNAL_RED_LED_PIN, 0);                                  // turn off red LED cause we're happy now
  Blynk.setProperty(BLYNK_CALIB_LED_PIN, "color", "#43D35C");             // change Blynk app's calibrating indicator LED to green
  digitalWrite(EXTERNAL_GREEN_LED_PIN, 1);                                // turn on green LED
//
//  q_calib = q_arm.getProduct(q_hand.getConjugate());                      // calculate rotation of hand frame to arm frame
//  q_calib.normalize();

  lcdPrint(0, "Calibration done!");
  lcdPrint(1, " ");
  delay(4000);
  Blynk.setProperty(BLYNK_START_CALIB_PIN, "offLabel", "Start");

  calibrating = false;
}

void updateIMUs()
{
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount1 < packet1Size) fifoCount1 = imu1.getFIFOCount();

    // read a packet from FIFO
    imu1.getFIFOBytes(fifoBuffer1, packet1Size);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount1 -= packet1Size;
    
    // display quaternion values in easy matrix form: w x y z
    imu1.dmpGetQuaternion(&q_arm, fifoBuffer1);

    while (fifoCount2 < packet2Size) fifoCount2 = imu2.getFIFOCount();
    
    imu2.getFIFOBytes(fifoBuffer2, packet2Size);
    fifoCount2 -= packet2Size;

    imu2.dmpGetQuaternion(&q_hand, fifoBuffer2);

// to print out quat values
// 
//        Serial.print("quat1\t");
//        Serial.print(q_arm.w);
//        Serial.print("\t");
//        Serial.print(q_arm.x);
//        Serial.print("\t");
//        Serial.print(q_arm.y);
//        Serial.print("\t");
//        Serial.println(q_arm.z);
//
//        Serial.print("quat2\t");
//        Serial.print(q_hand.w);
//        Serial.print("\t");
//        Serial.print(q_hand.x);
//        Serial.print("\t");
//        Serial.print(q_hand.y);
//        Serial.print("\t");
//        Serial.println(q_hand.z);

//  imu1.resetFIFO();
//  imu2.resetFIFO();
}
