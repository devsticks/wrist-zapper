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

#include "I2Cdev.h"                                 // helper functions for IMU interfacing
#include "MPU6050_6Axis_MotionApps_V6_12.h"         // functions specific to the MPU6050
#include "SD.h"                                     // SD card interfacing
//#include "Quaternion.h"
#include "helper_3dmath.h"                          // quaternion math
#include "BasicLinearAlgebra.h"                     // matrix math
#include "driver/ledc.h"                            // PWM control
#include "Wire.h"                                   // nice GPIO functions


#include "BlynkSimpleEsp32_BLE.h"
#include "BLEDevice.h"
#include "BLEServer.h"

//// -- CONSTANT DEFINITIONS -- ////

using namespace BLA;                                // BasicLinearAlgebra namespace

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

#define BLYNK_USE_DIRECT_CONNECT
/* Comment this out to disable Blynk prints and save space */
#define BLYNK_PRINT Serial

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

// orientation / motion vars
Quaternion q_arm, q_hand, q_calib;                  // [w, x, y, z] quaternion containers for arm, hand IMUs, and calibration between them
float extensionAngle = 0;                           // wrist extension angle

// Blynk vars
BlynkTimer fastTimer, slowTimer;
WidgetLCD lcd(BLYNK_LCD_PIN);                       // init Blynk LCD variable
WidgetLED sdLED(BLYNK_SD_LED_PIN);                  // init Blynk SD indicator LED
WidgetLED calibLED(BLYNK_CALIB_LED_PIN);            // init Blynk calibration indicator LED
// Auth Token for the microcontroller device.
// Go to Project Settings (nut icon) in the Blynk App to generate one.
char auth[] = "fMTNpL-rwcWIsqoN3n3SwtQAUDk7_YZg";   // Tom's ESP32
bool slowTicToc = false;

//// -- TIMER FUNCTIONS -- ////

// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void fastTimerEvent()
{
    // You can send any value at any time.
    // Please don't send more that 10 values per second.
    Blynk.virtualWrite(BLYNK_EXT_ANGLE_PIN, extensionAngle);
    Blynk.virtualWrite(BLYNK_BATTERY_PIN, getBatteryVoltage());

    if (extensionAngle >= 30) {
      Blynk.virtualWrite(BLYNK_EXT_ANGLE_IMG_PIN, 4);
    } else if (extensionAngle >= 20) {
      Blynk.virtualWrite(BLYNK_EXT_ANGLE_IMG_PIN, 3);
    } else if (extensionAngle >= 10) {
      Blynk.virtualWrite(BLYNK_EXT_ANGLE_IMG_PIN, 2);
    } else {
      Blynk.virtualWrite(BLYNK_EXT_ANGLE_IMG_PIN, 1);
    }
}

void slowTimerEvent() 
{
    slowTicToc != slowTicToc;
    if (!calibrating) 
    {
      if (getBatteryVoltage() > 10.5) // above 3.5V per cell
      {
        lcdPrint(0, "Angle: " + String(extensionAngle));
        lcdPrint(1, "Shock at: " + String(shockPercentage) + "%");
      } else {  // battery is dead (less than 3.5V per cell, panic)
        lcdPrint(0, "Battery depleted");
        lcdPrint(1, "Replace now");
      }
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
    slowTimer.setInterval(3000L, slowTimerEvent);
    
    // This will print Blynk Software version to the Terminal Widget when
    // your hardware gets connected to Blynk Server
    Serial.println(F("Blynk v" BLYNK_VERSION ": Device started"));
    Serial.println(F("-------------"));
    Serial.println(F("Type 'Marco' and get a reply, or type"));
    Serial.println(F("anything else and get it printed back."));
    Serial.flush();

    lcdPrint(0, "Device online");
    lcdPrint(1, "Hello!");

    setupGPIO();
    initializeIMUs();
    testIMUConnections();

    setupIMU(imu1, mpu1IntStatus, dmp1Ready, packet1Size, "IMU 1");
    setupIMU(imu2, mpu2IntStatus, dmp2Ready, packet2Size, "IMU 2");
    setupSD();

    if (devStatus != 0 || !dmp1Ready || ! dmp2Ready) { /* startupFailed() TODO turn a red light on or something to show there's a problem */ }

    toggleLED(externalRedLEDState, EXTERNAL_RED_LED_PIN);
    toggleLED(externalGreenLEDState, EXTERNAL_GREEN_LED_PIN);

    delay(2000); // let the IMUs settle before calibrating; may need to be longer
    calibrate();
}

void appendFile(fs::FS &fs, const char * path, /*const char * */ String message)
{
//    Serial.printf("Appending to file: %s\n", path);
    bool flag = true;                                                   // assume all is well

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
        flag = false;                                                   // all is actually not well
    } else if (file.print(message)) {
        Serial.printf("Message appended to file: %s\n", path);
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

void loop() {
    Blynk.run();
    fastTimer.run(); // Initiates BlynkTimer
    slowTimer.run(); 
  
  // Check for new data in the IMU FIFO's
//  if ( imu1.fifoAvailable() || imu2.fifoAvailable() )
//  {
//    imu1.dmpUpdateFifo();
//    imu2.dmpUpdateFifo();
//    //printIMUData();
//    Quaternion q_arm = getQuat(imu1);
//    Quaternion q_hand = getQuat(imu2);
//    float extensionAngle = calcExtensionAngle(q_arm, q_hand);
//    updateShock(extensionAngle);
//  }

//    // wait for MPU interrupt or extra packet(s) available
//    while (!mpu1Interrupt && fifoCount1 < packet1Size) {
//        if (mpu1Interrupt && fifoCount1 < packet1Size) {
//          // try to get out of the infinite loop 
//          fifoCount1 = imu1.getFIFOCount();
//        }  
//        // other program behavior stuff here
//        // .
//        // .
//        // .
//        // if you are really paranoid you can frequently test in between other
//        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
//        // while() loop to immediately process the MPU data
//        // .
//        // .
//        // .
//    }

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
        updateIMUs();                                             // get fresh quaternion orientations

        extensionAngle = calcExtensionAngle(q_arm, q_hand); 
        appendFile(SD, "/angle_log.txt", String(extensionAngle));
        
        Serial.println(String(extensionAngle));
        updateShock(extensionAngle);
        
//        Serial.println(String(getBatteryPercentage()));

        // blink internal LED to indicate activity
        toggleLED(internalLEDState, INTERNAL_LED_PIN);
    }
}
