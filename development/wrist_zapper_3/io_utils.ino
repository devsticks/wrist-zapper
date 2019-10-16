/* 
 *  A set of utility functions for GPIO and interfacing with an SD card
 */

/* 
 *  Mount the card
 */
void setupSD(void)
{
  //setup SD card
  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      return;
  }
  
  uint8_t cardType = SD.cardType();
}

/*
 *  Appends a line of text to a file specified in path parameter.
 *  Will create the file if it doesn't exist.
 *  
 *  Params:
 *  - &fs: an SD (or other) card object
 *  - path: the absolute path to the file to write to
 *  - message: the string to be appended
 */
void appendFile(fs::FS &fs, const char * path, /*const char * */ String message)
{
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

/*
 * 
 *  Params:
 *  - int stimRangeStart = 30; // start stimulus at this point
 *  - int stimRange = 30; // range over which intensity varies
 *  - float stimIntensityStart = 0; // percentage intensity at stimRangeStart
 *  - float stimIntensityRange = 2; // increase in percentage intensity from stimRangeStart to end of stimRange
 */
void updateShock(float extensionAngle, int stimRangeStart, int stimRange, float stimIntensityStart, float stimIntensityRange)
{
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
 }
