/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************

  You can send/receive any data using WidgetTerminal object.

  App project setup:
    Terminal widget attached to Virtual Pin V1
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <SparkFunMPU9250-DMP.h>

#ifdef defined(SAMD)
 #define SerialPort SerialUSB
#else
  #define SerialPort Serial
#endif

// imu1 = forearm, imu2 = hand
MPU9250_DMP imu1, imu2;

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
//char auth[] = "SBy_sPo-wRu5TSaNUWzvMsRytfc-j6J1"; //old esp32
char auth[] = "TMF-CRmP9gVt5I3jeyRgVCUHZ0Ssm-U0"; //huzzah32

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Cheeky Nandos";
//char pass[] = "canttouchthis";
char ssid[] = "Devin's iPad";
char pass[] = "john21:25";

BlynkTimer timer;

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void myTimerEvent()
{
  // dataReady() checks to see if new accel/gyro data
  // is available. It will return a boolean true or false
  // (New magnetometer data cannot be checked, as the library
  //  runs that sensor in single-conversion mode.)
  if ( imu1.dataReady() )
  {
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu1.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMU1Data();
  }

  if ( imu2.dataReady() )
  {
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu2.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMU2Data();
  }
    
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V1, imu1.calcAccel(imu1.ax));
  Blynk.virtualWrite(V2, imu2.calcAccel(imu2.ax));
}

void printIMU1Data(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX1 = imu1.calcAccel(imu1.ax);
  float accelY1 = imu1.calcAccel(imu1.ay);
  float accelZ1 = imu1.calcAccel(imu1.az);
  float gyroX1 = imu1.calcGyro(imu1.gx);
  float gyroY1 = imu1.calcGyro(imu1.gy);
  float gyroZ1 = imu1.calcGyro(imu1.gz);
  float magX1 = imu1.calcMag(imu1.mx);
  float magY1 = imu1.calcMag(imu1.my);
  float magZ1 = imu1.calcMag(imu1.mz);
  
  SerialPort.println("Accel1: " + String(accelX1) + ", " +
              String(accelY1) + ", " + String(accelZ1) + " g");
  SerialPort.println("Gyro1: " + String(gyroX1) + ", " +
              String(gyroY1) + ", " + String(gyroZ1) + " dps");
  SerialPort.println("Mag1: " + String(magX1) + ", " +
              String(magY1) + ", " + String(magZ1) + " uT");
  SerialPort.println("Time1: " + String(imu1.time) + " ms");
  SerialPort.println();
}

void printIMU2Data(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX2 = imu2.calcAccel(imu2.ax);
  float accelY2 = imu2.calcAccel(imu2.ay);
  float accelZ2 = imu2.calcAccel(imu2.az);
  float gyroX2 = imu2.calcGyro(imu2.gx);
  float gyroY2 = imu2.calcGyro(imu2.gy);
  float gyroZ2 = imu2.calcGyro(imu2.gz);
  float magX2 = imu2.calcMag(imu2.mx);
  float magY2 = imu2.calcMag(imu2.my);
  float magZ2 = imu2.calcMag(imu2.mz);
  
  SerialPort.println("Accel2: " + String(accelX2) + ", " +
              String(accelY2) + ", " + String(accelZ2) + " g");
  SerialPort.println("Gyro2: " + String(gyroX2) + ", " +
              String(gyroY2) + ", " + String(gyroZ2) + " dps");
  SerialPort.println("Mag2: " + String(magX2) + ", " +
              String(magY2) + ", " + String(magZ2) + " uT");
  SerialPort.println("Time: " + String(imu2.time) + " ms");
  SerialPort.println();
}

// Attach virtual serial terminal to Virtual Pin V1
WidgetTerminal terminal(V1);

// You can send commands from Terminal to your hardware. Just use
// the same Virtual Pin as your Terminal Widget
BLYNK_WRITE(V1)
{

  // if you type "Marco" into Terminal Widget - it will respond: "Polo:"
  if (String("Marco") == param.asStr()) {
    terminal.println("You said: 'Marco'") ;
    terminal.println("I said: 'Polo'") ;
  } else {

    // Send it back
    terminal.print("You said:");
    terminal.write(param.getBuffer(), param.getLength());
    terminal.println();
  }

  // Ensure everything is sent
  terminal.flush();
}

void setup()
{
  // Debug console, baud rate 115200 Hz
    SerialPort.begin(115200);

//  // Setup for second I2C interface for IMU2
//  Wire.begin(); // defaults to SCL,SDA,100000hz, SCL and SDA are defined in your Board Variants pins_arduino.h file.
//  Wire1.begin(SDA2,SCL2,100000); // there are no defined pins for the second peripheral.

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu1.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with IMU1");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  if (imu2.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with IMU2");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu1.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu2.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu1.setGyroFSR(2000); // Set gyro to 2000 dps
  imu2.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu1.setAccelFSR(2); // Set accel to +/-2g
  imu2.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu1.setLPF(5); // Set LPF corner frequency to 5Hz
  imu2.setLPF(5); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu1.setSampleRate(10); // Set sample rate to 10Hz
  imu2.setSampleRate(10); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu1.setCompassSampleRate(10); // Set mag rate to 10Hz
  imu2.setCompassSampleRate(10); // Set mag rate to 10Hz

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
  
  // This will print Blynk Software version to the Terminal Widget when
  // your hardware gets connected to Blynk Server
  terminal.println(F("Blynk v" BLYNK_VERSION ": Device started"));
  terminal.println(F("-------------"));
  terminal.println(F("Type 'Marco' and get a reply, or type"));
  terminal.println(F("anything else and get it printed back."));
  terminal.flush();
}

void loop()
{
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
}
