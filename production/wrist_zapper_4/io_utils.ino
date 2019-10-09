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
  if (!SD.begin())
  {
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
 *  - &fs: reference to an SD (or other) card object
 *  - path: the absolute path to the file to write to
 *  - message: the string to be appended
 */
void appendFile(fs::FS &fs, const char * path, /*const char * */ String message)
{
//    Serial.printf("Appending to file: %s\n", path);
    bool flag = true;                                                   // assume all is well

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
        flag = false;                                                   // all is actually not well
    } else if (file.print(message)) {
//        Serial.printf("Message appended to file: %s\n", path);
    } else {
        Serial.println("Append failed");
        flag = false;                                                   // all definitely isn't well
        Blynk.setProperty(BLYNK_SD_LED_PIN, "color", "#D3435C");        // make Blynk app's SD indicator LED red
    }
    file.close();

    // change LED indicator colour if necessary
    if (flag && sdStatus) // all is good, we knew it was so
    {
        if (!calibrating) {
            toggleLED(externalGreenLEDState, EXTERNAL_GREEN_LED_PIN);   // blink green LED to show we're writing happily
        } // else if calibrating, do nothing
    } 
    else if (flag && !sdStatus)  // what was wrong has just come right 
    {
        sdStatus = true;
        Blynk.setProperty(BLYNK_SD_LED_PIN, "color", "#43D35C");        // make Blynk app's SD indicator LED green
    }
    else if (!flag && sdStatus) // something's just gone wrong
    {
        sdStatus = false;
        Blynk.setProperty(BLYNK_SD_LED_PIN, "color", "#D3435C");        // change Blynk app's SD indicator LED red
        digitalWrite(EXTERNAL_GREEN_LED_PIN, 0);                        // turn the green LED off
    }
    else // something's gone wrong, but we knew this already
    {
        digitalWrite(EXTERNAL_GREEN_LED_PIN, 0);                        // turn the green LED off
    } 
}

//void saveToSD(void)
//{
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
//}

// convert a percentage of max current to a DAC value
int toDac(int percentage)
{
  float minVoltage = 0.4; // threshold voltage of BJT
  float maxVoltage = 3.12; // max produceable voltage
  int dacMaxVal = 255;
float outputVoltage = 0.6;
//  float outputVoltage = ((percentage * 0.01) * (maxVoltage - minVoltage)) + minVoltage; // calculate the output voltage we want
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

  if (calibrating || extensionAngle < shockStartAngle) // don't shock if we're out of range or calibrating
  {
//      ledcWrite(STIM_PIN,0);
      intensity = 0;
      dacWrite(STIM_PIN,0);
      Serial.println("Shock off");
  }
  else if (extensionAngle > shockStartAngle + shockAngleRange) // max intensity
  { 
    intensity = shockStartIntensity + shockIntensityRange;
    int j = toDac(intensity);
    dacWrite(0,j);
    Serial.println("Shocking at max intensity, " + String(intensity) + "%!");
  } 
  else // normal shock range
  {
    intensity = shockStartIntensity + (extensionAngle - shockStartAngle)/(shockAngleRange / shockIntensityRange);
//      for(int i=0;i<100;i+=10) {
    int j = toDac(intensity);
//    int j = intensity * 8192 * 0.01;

//    ledcWrite(STIM_PIN, j);

    dacWrite(STIM_PIN,j);
//    Serial.println("Shocking at " + String(intensity) + "%!");
//        delay(2000);            
//      } 
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
 *  @ description:
 *      - Takes the ADC reading from the pin attached to the battery voltage divider
 *      - ADC reads 0-4095, over 0 to 3.3V
 *      - Max voltage (12V) will read 3V, ie ~3722
 *  @ return: float - the battery voltage
 */
float getBatteryVoltage() 
{
  float ADCVal = analogRead(BATTERY_TEST_PIN);
  float batteryVoltage = (ADCVal * 4 * 3) / 3722;
//  Serial.println("Battery voltage: " + String(batteryVoltage));
  return batteryVoltage;
}

/* 
 *  getBatteryPercentage
 *  
 */
float getBatteryPercentage()
{
  return 100 * ((getBatteryVoltage() - 10.5) / 1.5); // TODO get the actual battery percentage...
}

/*
 *  lcdPrint
 *  
 *  @ description: writes a line to the Blynk app lcd screen
 *  @ params: 
 *      - line - zero-indexed line number, 0 or 1
 *      - message - a string to be written; will be truncated at 16 characters 
 */
void lcdPrint(char line, String message)
{
  while (message.length() <= 16) {
    message += " ";
  }
  lcd.print(0, line, message);
}

/*
 *  BLYNK_CONNECTED
 *  
 *  @ description: run when device connects to Blynk app
 */
BLYNK_CONNECTED() 
{
  lcdPrint(0, "Device online");
  lcdPrint(1, "Hello!");
  
  Blynk.virtualWrite(BLYNK_SHOCK_START_ANGLE_PIN, shockStartAngle);
  Blynk.virtualWrite(BLYNK_SHOCK_MAX_ANGLE_PIN, shockMaxAngle);
  Blynk.virtualWrite(BLYNK_SHOCK_START_INTENSITY_PIN, shockStartIntensity);
  Blynk.virtualWrite(BLYNK_SHOCK_MAX_INTENSITY_PIN, shockMaxIntensity);

  rtc.begin(); // sync time with Blynk real time clock
}

/*
 *  BLYNK_WRITE - Start calibration
 *  
 *  @ description: run when Calibrate button is pressed in Blynk app
 */
BLYNK_WRITE(BLYNK_START_CALIB_PIN) 
{
  if (param.asInt() == 1) // start calibration on down press of calibration button
  {
    calibLED.on();
    Serial.println("Calibration requested");
    calibrate();
    calibLED.off();
  }
}

/*
 *  BLYNK_WRITE - Change shock start angle
 *  
 *  @ description: updates when shock start angle slider is altered in Blynk app
 */
BLYNK_WRITE(BLYNK_SHOCK_START_ANGLE_PIN) 
{
  shockStartAngle = param.asInt();
}

/*
 *  BLYNK_WRITE - Change shock max angle
 *  
 *  @ description: updates when shock max angle slider is altered in Blynk app
 */
BLYNK_WRITE(BLYNK_SHOCK_MAX_ANGLE_PIN) 
{
  shockMaxAngle = param.asInt();
}

/*
 *  BLYNK_WRITE - Change shock start intensity
 *  
 *  @ description: updates when shock start intensity is altered in Blynk app
 */
BLYNK_WRITE(BLYNK_SHOCK_START_INTENSITY_PIN) 
{
  shockStartIntensity = param.asInt();
}

/*
 *  BLYNK_WRITE - Change shock max intensity
 *  
 *  @ description: updates when shock max intensity is altered in Blynk app
 */
BLYNK_WRITE(BLYNK_SHOCK_MAX_INTENSITY_PIN) 
{
  shockMaxIntensity = param.asInt();
}

/*
 *  getDateTimeString
 *  
 *  @ description: returns a formatted string of the current system time (which defaults to 1970 if not set... See timeStatus() )
 */
String getDateTimeString()
{
  return String(day()) + "/" + month() + "/" + year() + ", " + String(hour()) + ":" + minute() + ":" + second();
}
