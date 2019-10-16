/************************************************************
 * 
 *  https://github.com/devsticks/wrist-zapper
 *  
 *  @ description: code for a device which uses two IMUs to calculate the extension angle of 
 *                 a wearer's wrist, and shock them when they enter a set range of movement
 *  @ author: Devin Stickells
 *  @ license: GNU General Public License v3.0
 *  @ supported devices: Espressif ESP32 (Tested on Adafruit Feather Huzzah32)
 */

//// -- INCLUDES -- ////

#include "BlynkSimpleEsp32_BLE.h"                   // https://github.com/blynkkk/blynk-library
#include "BLEDevice.h"
#include "BLEServer.h"

#include "I2Cdev.h"                                 // helper functions for IMU interfacing https://github.com/jrowberg/i2cdevlib
#include "MPU6050_6Axis_MotionApps_V6_12.h"         // functions specific to the MPU6050, from I2CDevLib https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "SD.h"                                     // SD card interfacing
#include "TimeLib.h"                                // Paul Stoffregen's Arduino time library https://github.com/PaulStoffregen/Time
#include "WidgetRTC.h"                              // to get real time from Blynk
#include "helper_3dmath.h"                          // quaternion math, part of I2CDevLib https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "BasicLinearAlgebra.h"                     // matrix math, https://github.com/tomstewart89/BasicLinearAlgebra
#include "driver/ledc.h"                            // PWM control
#include "Wire.h"                                   // nice GPIO functions

//// -- CONSTANT DEFINITIONS -- ////

using namespace BLA;                                // BasicLinearAlgebra namespace
#define BATTERY_CUTOFF_VOLTAGE 10.5                 // Voltage at which the device starts to complain about a low battery

// hardware pins
#define INTERNAL_LED_PIN 13 
#define EXTERNAL_RED_LED_PIN 15
#define EXTERNAL_GREEN_LED_PIN 14
#define BATTERY_TEST_PIN 32                         // analog input from battery voltage divider
#define STIM_PIN 25                                 // analog output to base of stim control BJT
#define SDA_PIN 22                                  // I2C SDA pin
#define SCL_PIN 23                                  // I2C SCL pin

// Blynk virtual pins
#define BLYNK_EXT_ANGLE_PIN V1
#define BLYNK_BATTERY_PIN V2
#define BLYNK_LCD_PIN V3
#define BLYNK_SD_LED_PIN V4
#define BLYNK_EXT_ANGLE_IMG_PIN V5
#define BLYNK_START_CALIB_PIN V6
#define BLYNK_CALIB_LED_PIN V7
#define BLYNK_SHOCK_START_ANGLE_PIN V8
#define BLYNK_SHOCK_MAX_ANGLE_PIN V9
#define BLYNK_SHOCK_START_INTENSITY_PIN V10
#define BLYNK_SHOCK_MAX_INTENSITY_PIN V11
#define TEST_PIN V12
#define BLYNK_SHOCK_START_IMG_PIN V13
#define BLYNK_SHOCK_MAX_IMG_PIN V14

#define BLYNK_USE_DIRECT_CONNECT
/* Comment this out to disable Blynk prints and save space */
#define BLYNK_PRINT Serial

float testvar = 0;

//// -- GLOBAL VARIABLES -- ////

// IMU AD0 pin low = 0x68 (hand IMU)
// IMU AD0 pin high = 0x69 (arm IMU)
MPU6050 imu1(0x69), imu2(0x68);                     // initialise IMU objects with their I2C addresses

// LED state vars for toggle / blinking functionality
bool internalLEDState = false;
bool externalRedLEDState = false;
bool externalGreenLEDState = false;

// IMU control / status vars
bool dmp1Ready = false, dmp2Ready = false;          // flag set true if DMP init was successful
uint8_t mpu1IntStatus, mpu2IntStatus;               // holds actual interrupt status byte from IMU
uint8_t devStatus;                                  // return status after each device operation (0 = success, !0 = error)
uint16_t packet1Size, packet2Size;                  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1, fifoCount2;                    // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64], fifoBuffer2[64];           // FIFO storage buffer
volatile bool mpu1Interrupt = false;                // indicates whether MPU1 interrupt pin has gone high
volatile bool mpu2Interrupt = false;                // indicates whether MPU2 interrupt pin has gone high
float shockPercentage = 0;
char shockStartAngle = 30, shockMaxAngle = 60;      // wrist angle range in which to shock wearer (in degrees upward from horizontal)
char shockStartIntensity = 0, shockMaxIntensity = 10; // variation in shock intensity over critical range - corresponding to start and max angles
bool calibrating = false;
bool calibrated = false;
bool sdStatus = true;
String logFileName = "/angle_log.txt";

// orientation / motion vars
Quaternion q_arm, q_hand, q_calib;                  // [w, x, y, z] quaternion containers for arm, hand IMUs, and calibration between them
float extensionAngle = 0;                           // wrist extension angle

// Blynk vars
BlynkTimer fastTimer, slowTimer;
WidgetLCD lcd(BLYNK_LCD_PIN);                       // init Blynk LCD variable
WidgetLED sdLED(BLYNK_SD_LED_PIN);                  // init Blynk SD indicator LED
WidgetLED calibLED(BLYNK_CALIB_LED_PIN);            // init Blynk calibration indicator LED
WidgetRTC rtc;                                      // init Blynk real time clock object
// Auth Token for the microcontroller device.
// Go to Project Settings (nut icon) in the Blynk App to generate one.
char auth[] = "fMTNpL-rwcWIsqoN3n3SwtQAUDk7_YZg";   // Tom's ESP32
bool slowTicToc = false;


//// -- FUNCTION PROTOTYPES -- ////
/* the Arduino IDE doesn't build function prototypes if they take references as arguments, so we have to make them ourselves... */

void appendFile(fs::FS &fs, String path, /*const char * */ String message);


//// -- TIMER FUNCTIONS -- ////

// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void fastTimerEvent() // 500ms - Battles with more that 10 values per second.
{
    Blynk.virtualWrite(BLYNK_EXT_ANGLE_PIN, extensionAngle);

    setWristImage(BLYNK_EXT_ANGLE_IMG_PIN, extensionAngle);

    // Blink the red light if battery is dead
    if (getBatteryVoltage() < BATTERY_CUTOFF_VOLTAGE)
    {
      toggleLED(externalRedLEDState, EXTERNAL_RED_LED_PIN);
    } 
    else if (externalRedLEDState && !calibrating) // if red LED is on and battery is fine, kill it
    {
      externalRedLEDState = false;
      digitalWrite(EXTERNAL_RED_LED_PIN, 0);
    }
}

void slowTimerEvent() // 10s
{
  slowTicToc != slowTicToc;
  if (!calibrating) // don't overwrite instructions on LCD screen 
  {
    if (getBatteryVoltage() > BATTERY_CUTOFF_VOLTAGE) // above 10.5V, i.e. 3.5V per cell
    {
      if (devStatus != 0 || !dmp1Ready || ! dmp2Ready) 
      { /* startupFailed() */  
        lcdPrint(0, "Device error");
        lcdPrint(1, "Try a reboot"); 
      } else {
        lcdPrint(0, "Device online");
        lcdPrint(1, "Hello!");
      }
    } else {  // battery is dead (less than 3.5V per cell, panic)
      lcdPrint(0, "Battery depleted");
      lcdPrint(1, "Replace now");
    }
  }

  if (sdStatus) // mircoSD card is working fine
  {
    Blynk.setProperty(BLYNK_SD_LED_PIN, "color", "#43D35C");        // make Blynk app's SD indicator LED green
  }
  else
  {
    Blynk.setProperty(BLYNK_SD_LED_PIN, "color", "#D3435C");        // make Blynk app's SD indicator LED red
  }
}

//// -- SETUP -- ////
// runs when the device starts up
void setup() 
{
    Serial.begin(115200);                             // initialize serial comms   
  
    Serial.println("Waiting for connections...");
    Blynk.setDeviceName("Wrist Zapper");
    Blynk.begin(auth);

    // Setup a function to be called every second
    fastTimer.setInterval(500L, fastTimerEvent);
    slowTimer.setInterval(10000L, slowTimerEvent);
    
    // This will print Blynk Software version to the Terminal Widget when
    // your hardware gets connected to Blynk Server
    Serial.println(F("Blynk v" BLYNK_VERSION ": Device started"));
    Serial.println(F("-------------"));
    Serial.println(F("Type 'Marco' and get a reply, or type"));
    Serial.println(F("anything else and get it printed back."));
    Serial.flush();

    setSyncInterval(10 * 60); // update real time clock every 10 minutes

    setupGPIO();
    initializeIMUs();
    testIMUConnections();

    setupIMU(imu1, mpu1IntStatus, dmp1Ready, packet1Size, "IMU 1");
    setupIMU(imu2, mpu2IntStatus, dmp2Ready, packet2Size, "IMU 2");
    
    setupSD();

    if (devStatus != 0 || !dmp1Ready || ! dmp2Ready) 
    { /* startupFailed() */  
      lcdPrint(0, "Device error");
      lcdPrint(1, "Try a reboot"); 
      toggleLED(externalRedLEDState, EXTERNAL_RED_LED_PIN);   
    } 
    else // happy days
    {
      lcdPrint(0, "Device online");
      lcdPrint(1, "Hello!");
    }

//    // set wrist angle images in configuration tab of app
//    Blynk.virtualWrite(BLYNK_SHOCK_START_ANGLE_PIN, shockStartAngle);
//    setWristImage(BLYNK_SHOCK_START_IMG_PIN, shockStartAngle);
//    Blynk.virtualWrite(BLYNK_SHOCK_MAX_ANGLE_PIN, shockMaxAngle);
//    setWristImage(BLYNK_SHOCK_MAX_IMG_PIN, shockMaxAngle);
    
    // flash LEDs, we're awake!
    for (int i = 0; i < 5; i += 1) 
    {
      toggleLED(externalRedLEDState, EXTERNAL_RED_LED_PIN);
      toggleLED(externalGreenLEDState, EXTERNAL_GREEN_LED_PIN);
      delay(200);
    }

    delay(2000); // let things settle 
}

// The main hardware loop

void loop() {
    Blynk.run();
    fastTimer.run(); // Initiates BlynkTimer
    slowTimer.run(); 

    if (devStatus != 0 || !dmp1Ready || ! dmp2Ready) 
    { /* startupFailed() */  
      return;
    }

    // reset interrupt flag and get INT_STATUS byte
    mpu1Interrupt = false;
    mpu1IntStatus = imu1.getIntStatus();

    mpu2Interrupt = false;
    mpu2IntStatus = imu2.getIntStatus();

    // get current FIFO count
    fifoCount1 = imu1.getFIFOCount();
    fifoCount2 = imu2.getFIFOCount();

    bool overflow = false;

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpu1IntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount1 >= 1024) 
    {
        // reset so we can continue cleanly
        imu1.resetFIFO();
        Serial.println(F("FIFO 1 overflow!"));
         overflow = true;
    }
    
    if ((mpu2IntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount2 >= 1024) 
    {
        // reset so we can continue cleanly
        imu2.resetFIFO();
        Serial.println(F("FIFO 2 overflow!"));
        overflow = true;
    }
    
    if (overflow) 
    {
      overflow = false;
      return;
    }
//      otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpu1IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT) || mpu2IntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT) ) 
    {
        while (fifoCount1 > 100 || fifoCount2 > 100) updateIMUs();                                             // get fresh quaternion orientations

        extensionAngle = calcExtensionAngle(q_arm, q_hand); 

        // build date time string for easy conversion to CSV
        String dateString = "";
        switch (timeStatus()) { 
          case timeSet:
          {
               dateString = "Set, ";
               break;
          }
          case timeNeedsSync: 
          { 
             dateString = "Needs Sync, ";
             break;
          }
          default: 
          { 
             dateString = "Not Set, ";
             break;
          }
        }

        dateString += getDateTimeString();
        //"/angle_log.txt"
        appendFile(SD, logFileName, dateString + ", " + String(extensionAngle) + "\n");
        
        Serial.println(String(extensionAngle));
        updateShock(extensionAngle);
        
//        Serial.println(String(getBatteryPercentage()));

        // blink internal LED to indicate activity
        toggleLED(internalLEDState, INTERNAL_LED_PIN);
    }
}
