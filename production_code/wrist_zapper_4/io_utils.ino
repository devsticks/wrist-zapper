/* 
 *  @ description: A set of utility functions for GPIO and interfacing with an SD card
 *  @ author: Devin Stickells, https://github.com/devsticks
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
 *  Mount the SD card
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
 *  appendFile
 *  
 *  Description:
 *  - Appends a line of text to a file specified in path parameter.
 *  - Will create the file if it doesn't exist.
 *  
 *  Params:
 *  - &fs: reference to an SD (or other) card object
 *  - path: the absolute path to the file to write to
 *  - message: the string to be appended
 */
void appendFile(fs::FS &fs, String path, /*const char * */ String message)
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

// 
/*
 *  toDac
 *  
 *  Description:
 *  - converts a percentage of the max output current to a DAC value
 *  
 *  Params:
 *  - percentage: an integer between 0 and 100 to be converted to a voltage between 0.4 and and 3.1 - controls the taps on the BJT in the stimulus control circuitry
 */
int toDac(int percentage)
{
  float minVoltage = 0.4; // threshold voltage of BJT
  float maxVoltage = 3.12; // max produceable voltage
  int dacMaxVal = 255;
//float outputVoltage = testvar * 0.1;
//Serial.println(outputVoltage);
  float outputVoltage = ((percentage * 0.01) * (maxVoltage - minVoltage)) + minVoltage; // calculate the output voltage we want
  int dacVal = round(outputVoltage * (255 / maxVoltage)); // convert required voltage to a DAC value
  
  return dacVal;
}

/*
 *  updateShock
 *  
 *  Description:
 *  - Updates the shock being experienced by the wearer depending on the extension angle of their wrist and the device configuration
 * 
 *  Params:
 *  - int shockStartAngle = 30; // start stimulus at this point
 *  - int shockAngleRange = 30; // range over which intensity varies
 *  - float shockStartIntensity = 0; // percentage intensity at shockStartAngle
 *  - float shockIntensityRange = 2; // increase in percentage intensity from shockStartAngle to end of shockAngleRange
 *  
 */
void updateShock(float extensionAngle)
{

  int shockAngleRange = shockMaxAngle - shockStartAngle;
  int shockIntensityRange = shockMaxIntensity - shockStartIntensity;
  float intensity = 0;

  if (calibrating || extensionAngle < shockStartAngle || !calibrated) // don't shock if we're out of range or are calibrating or haven't calibrated yet
  {
//      ledcWrite(STIM_PIN,0); // Testing using PWM
      intensity = 0;
      dacWrite(STIM_PIN,0);
//      Serial.println("Shock off");
  }
  else if (extensionAngle > shockStartAngle + shockAngleRange) // max intensity
  { 
    intensity = shockStartIntensity + shockIntensityRange;
    int j = toDac(intensity);
    dacWrite(0,j);
//    Serial.println("Shocking at max intensity, " + String(intensity) + "%!");
  } 
  else // normal shock range
  {
    intensity = shockStartIntensity + (extensionAngle - shockStartAngle)/(shockAngleRange / shockIntensityRange);
    int j = toDac(intensity);

//// Testing using PWM
//    int j = intensity * 8192 * 0.01;
//    ledcWrite(STIM_PIN, j);

    dacWrite(STIM_PIN,j);
//    Serial.println("Shocking at " + String(intensity) + "%!");
  } 

  shockPercentage = intensity;
}

/* 
 *  toggleLED
 *  
 *  Description:
 *  - Toggles the state of an LED pin given a reference to it's state variable and its pin number
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
 *  Description:
 *  - Returns an estimate of the battery's percentage charge.  
 *  - Assumes a linear discharge down to 3.5V per cell. This isn't true.
 *  - The battery indicator on the app circumnavigates this problem by just showing the actual battery voltage on a slider, where 10.5 is dead and 12 is full.
 */
float getBatteryPercentage()
{
  return 100 * ((getBatteryVoltage() - 10.5) / 1.5); 
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

  // set wrist angle images and sliders in configuration tab of app
  Blynk.virtualWrite(BLYNK_SHOCK_START_ANGLE_PIN, shockStartAngle);
  setWristImage(BLYNK_SHOCK_START_IMG_PIN, shockStartAngle);
  Blynk.virtualWrite(BLYNK_SHOCK_MAX_ANGLE_PIN, shockMaxAngle);
  setWristImage(BLYNK_SHOCK_MAX_IMG_PIN, shockMaxAngle);
  Blynk.virtualWrite(BLYNK_SHOCK_START_INTENSITY_PIN, shockStartIntensity);
  Blynk.virtualWrite(BLYNK_SHOCK_MAX_INTENSITY_PIN, shockMaxIntensity);

  if (devStatus != 0 || !dmp1Ready || ! dmp2Ready) 
    { /* startupFailed() */  
      lcdPrint(0, "Device error");
      lcdPrint(1, "Try a reboot"); 
      toggleLED(externalRedLEDState, EXTERNAL_RED_LED_PIN);   
    }
}

/*
 *  BLYNK_WRITE - Start calibration
 *  
 *  @ description: run when Calibrate button is pressed in Blynk app
 */
BLYNK_WRITE(BLYNK_START_CALIB_PIN) 
{
  if (param.asInt() == 1 && !calibrating) // start calibration on down press of calibration button
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
  if (shockStartAngle > shockMaxAngle) // make sure max angle is never less than start angle
  {
    shockStartAngle = shockMaxAngle;
    Blynk.virtualWrite(BLYNK_SHOCK_START_ANGLE_PIN, shockStartAngle);
  }
  setWristImage(BLYNK_SHOCK_START_IMG_PIN, shockStartAngle);
}

/*
 *  BLYNK_WRITE - Change shock max angle
 *  
 *  @ description: updates when shock max angle slider is altered in Blynk app
 */
BLYNK_WRITE(BLYNK_SHOCK_MAX_ANGLE_PIN) 
{
  shockMaxAngle = param.asInt();
  if (shockMaxAngle < shockStartAngle) // make sure max angle is never less than start angle
  {
    shockMaxAngle = shockStartAngle;
    Blynk.virtualWrite(BLYNK_SHOCK_MAX_ANGLE_PIN, shockMaxAngle);
  }
  setWristImage(BLYNK_SHOCK_MAX_IMG_PIN, shockMaxAngle);
}

/*
 *  BLYNK_WRITE - Change shock start intensity
 *  
 *  @ description: updates when shock start intensity is altered in Blynk app
 */
BLYNK_WRITE(BLYNK_SHOCK_START_INTENSITY_PIN) 
{
  shockStartIntensity = param.asInt();
  if (shockMaxIntensity < shockStartIntensity) // make sure max intensity is never less than start intensity
  {
    shockStartIntensity = shockMaxIntensity;
    Blynk.virtualWrite(BLYNK_SHOCK_START_INTENSITY_PIN, shockStartIntensity);
  }
}

/*
 *  BLYNK_WRITE - Change shock max intensity
 *  
 *  @ description: updates when shock max intensity is altered in Blynk app
 */
BLYNK_WRITE(BLYNK_SHOCK_MAX_INTENSITY_PIN) 
{
  shockMaxIntensity = param.asInt();
  if (shockMaxIntensity < shockStartIntensity) // make sure max intensity is never less than start intensity
  {
    shockMaxIntensity = shockStartIntensity;
    Blynk.virtualWrite(BLYNK_SHOCK_MAX_INTENSITY_PIN, shockMaxIntensity);
  }
}

/*
 *  BLYNK_READ - Read the battery voltage
 *  
 *  @ description: reads and returns the battery voltage when requested to by the Blynk app
 */
BLYNK_READ(BLYNK_BATTERY_PIN) 
{
  Blynk.virtualWrite(BLYNK_BATTERY_PIN, getBatteryVoltage());

  if (getBatteryVoltage() < BATTERY_CUTOFF_VOLTAGE) // below 10.5V, i.e. 3.5V per cell, battery dead
  {
    lcdPrint(0, "Battery depleted");
    lcdPrint(1, "Replace now");
  }
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

/*
 *  setWristImage
 *  
 *  @ description: utility function to update the relevant wrist angle image in the app when needed
 */
void setWristImage(uint8_t pin, float angle)
{
    if (angle >= 90) {
      Blynk.virtualWrite(pin, 10);
    } else if (angle >= 80) {
      Blynk.virtualWrite(pin, 9);
    } else if (angle >= 70) {
      Blynk.virtualWrite(pin, 8);
    } else if (angle >= 60) {
      Blynk.virtualWrite(pin, 7);
    } else if (angle >= 50) {
      Blynk.virtualWrite(pin, 6);
    } else if (angle >= 40) {
      Blynk.virtualWrite(pin, 5);
    } else if (angle >= 30) {
      Blynk.virtualWrite(pin, 4);
    } else if (angle >= 20) {
      Blynk.virtualWrite(pin, 3);
    } else if (angle >= 10) {
      Blynk.virtualWrite(pin, 2);
    } else if (angle > -10) {
      Blynk.virtualWrite(pin, 1);
    } else if (angle > -20) {
      Blynk.virtualWrite(pin, 11);
    } else if (angle > -30) {
      Blynk.virtualWrite(pin, 12);
    } else if (angle > -40) {
      Blynk.virtualWrite(pin, 13);
    } else if (angle > -50) {
      Blynk.virtualWrite(pin, 14);
    } else if (angle > -60) {
      Blynk.virtualWrite(pin, 15);
    } else if (angle > -70) {
      Blynk.virtualWrite(pin, 16);
    } else if (angle > -80) {
      Blynk.virtualWrite(pin, 17);
    } else if (angle > -90) {
      Blynk.virtualWrite(pin, 18);
    } else {
      Blynk.virtualWrite(pin, 19);
    } 
}
