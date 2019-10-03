/* 
 *  @ description: A set of utility functions for GPIO and interfacing with an SD card
 *  @ author: Devin Stickells, https://github.com/devsticks
 *  @ required: 
 */

/*
 *  Set pin states
 */
void setupGPIO(void)
{
    Wire.begin(SDA_PIN, SCL_PIN);                     // start I2C interface with Wire library                  
  
    // configure LED pins for output
    pinMode(INTERNAL_LED_PIN, OUTPUT);
    pinMode(EXTERNAL_RED_LED_PIN, OUTPUT);
    pinMode(EXTERNAL_GREEN_LED_PIN, OUTPUT);    

    // configure stim pin for PWM - (pin number, PWM channel)
//    ledcAttachPin(STIM_PIN, 0);

// TODO check if this is actually needed (from i2cdevlib)
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    sdLED.on();
}  

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
//void appendFile(fs::FS &fs, const char * path, /*const char * */ String message)
//{
//    Serial.printf("Appending to file: %s\n", path);
//
//    File file = fs.open(path, FILE_APPEND);
//    if(!file){
//        Serial.println("Failed to open file for appending");
//        return;
//    }
//    if(file.print(message)){
//        Serial.println("Message appended");
//    } else {
//        Serial.println("Append failed");
//    }
//    file.close();
//}

void saveToSD(void)
{
//    float q1w = imu1.calcQuat(imu1.qw);
//    float q1x = imu1.calcQuat(imu1.qx);
//    float q1y = imu1.calcQuat(imu1.qy);
//    float q1z = imu1.calcQuat(imu1.qz);
//
//    float q2w = imu2.calcQuat(imu2.qw);
//    float q2x = imu2.calcQuat(imu2.qx);
//    float q2y = imu2.calcQuat(imu2.qy);
//    float q2z = imu2.calcQuat(imu2.qz);
//    
//    //String headerString = "ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,ax2,ay2,az2,gx2,gy2,gz2,mx2,my2,mz2";
//    String IMUString = String(q1w) + "," + String(q1x) + "," + String(q1y) + "," + String(q1z) + "," + String(q2w) + "," + String(q2x) + "," + String(q2y) + "," + String(q2z) + "\n";
//    //const char * IMU1Char = (const char *)IMU1String;
//    //appendFile(SD, "/log.txt", headerString + "\n");
//    appendFile(SD, "/qlog.txt", IMUString);
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
 *  - int shockStartAngle = 30; // start stimulus at this point
 *  - int shockAngleRange = 30; // range over which intensity varies
 *  - float shockStartIntensity = 0; // percentage intensity at shockStartAngle
 *  - float shockIntensityRange = 2; // increase in percentage intensity from shockStartAngle to end of shockAngleRange
 */
void updateShock(float extensionAngle)
{

  int shockAngleRange = shockMaxAngle - shockStartAngle;
  int shockIntensityRange = shockMaxIntensity - shockStartIntensity;
  float intensity = 0;
  
  if (extensionAngle > shockStartAngle + shockAngleRange) // max intensity
  { 
    intensity = shockStartIntensity + shockIntensityRange;
    int j = toDac(intensity);
    dacWrite(0,j);
    Serial.println("Shocking at max intensity, " + String(intensity) + "%!");
  } 
  else if (extensionAngle > shockStartAngle) 
  {
    intensity = shockStartIntensity + (extensionAngle - shockStartAngle)/(shockAngleRange / shockIntensityRange);
//      for(int i=0;i<100;i+=10) {
    int j = toDac(intensity);
//    int j = intensity * 8192 * 0.01;

//    ledcWrite(STIM_PIN, j);

    dacWrite(STIM_PIN,j);
    Serial.println("Shocking at " + String(intensity) + "%!");
//        delay(2000);            
//      } 
  } else {
//      ledcWrite(STIM_PIN,0);
      intensity = 0;
      dacWrite(STIM_PIN,0);
      Serial.println("Shock off");
  }

  shockPercentage = intensity;
}

/* 
 *  Toggle the state of an LED pin given it's state variable and pin number
 */

void toggleLED(bool &ledState, int pin) 
{
  ledState = !ledState;
  digitalWrite(pin, ledState);
}

/* 
 *  getBatteryVoltage
 *  
 *  Get the ADC reading from the pin attached to the battery voltage divider
 *  ADC reads 0-4095
 *  Max voltage is 3.3V
 */
float getBatteryVoltage() 
{
  float ADCVal = analogRead(BATTERY_TEST_PIN);
  return ADCVal * 3.3 / 4095;
}

float getBatteryPercentage()
{
  return 100 * (getBatteryVoltage() / 3.3); // TODO get the actual battery percentage...
}

/*
 *  lcdPrint
 *  
 *  @ description: writes a line to the Blynk app lcd screen
 *  @ params: 
 *  - line - zero-indexed line number, 0 or 1
 *  - message - a string to be written; will be truncated at 16 characters 
 */
void lcdPrint(char line, String message)
{
  while (message.length() <= 16) {
    message += " ";
  }
  lcd.print(0, line, message);
}

BLYNK_CONNECTED() 
{
  Blynk.syncVirtual(BLYNK_START_CALIB_PIN);
}

BLYNK_WRITE(BLYNK_START_CALIB_PIN) 
{
  toggleLED(externalGreenLEDState, EXTERNAL_GREEN_LED_PIN);
  calibLED.on();
  Serial.println("Calibration requested");
  calibrate();
  calibLED.off();
}
