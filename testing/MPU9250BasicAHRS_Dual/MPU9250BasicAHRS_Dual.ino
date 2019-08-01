/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#include "Wire.h"   
#include "MPU9250.h"
#include <SPI.h>

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO_1 0
#define ADO_2 1
// Define I2C addresses of the two MPU9250
#define MPU9250_ADDRESS_1 0x68   // Device address when ADO = 0
#define MPU1              0x68
#define MPU9250_ADDRESS_2 0x69   // Device address when ADO = 1
#define MPU2              0x69
#define AK8963_ADDRESS    0x0C   //  Address of magnetometer

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

// Set initial input parameters
//enum Ascale {
//  AFS_2G = 0,
//  AFS_4G,
//  AFS_8G,
//  AFS_16G
//};
//
//enum Gscale {
//  GFS_250DPS = 0,
//  GFS_500DPS,
//  GFS_1000DPS,
//  GFS_2000DPS
//};
//
//enum Mscale {
//  MFS_14BITS = 0, // 0.6 mG per LSB
//  MFS_16BITS      // 0.15 mG per LSB
//};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
uint8_t sampleRate = 0x04;
  
// Pin definitions
int intPin1 = 11;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int intPin2 = 12;
int myLed = 13; // Set up pin 13 led for toggling

bool intFlag1 = false;
bool intFlag2 = false;
bool newMagData = false;

//int16_t accelCount1[3],accelCount2[3];  // Stores the 16-bit signed accelerometer sensor output
//int16_t gyroCount1[3],gyroCount2[3];   // Stores the 16-bit signed gyro sensor output
int16_t MPU9250Data1[7], MPU9250Data2[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount1[3],magCount2[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration1[3] = {0, 0, 0}, magCalibration2[3] = {0, 0, 0}, magBias1[3] = {0, 0, 0}, magBias2[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias1[3] = {0, 0, 0},gyroBias2[3] = {0, 0, 0}, accelBias1[3] = {0, 0, 0},accelBias2[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float magScale1[3]  = {1.01, 1.03, 0.96}, magScale2[3]  = {1.01, 1.03, 0.96};
int16_t tempCount1,tempCount2;      // temperature raw count output
float   temperature1,temperature2;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t1, delt_t2 = 0;                      // used to control display output rate
uint32_t count1 = 0, sumCount1 = 0, count2 = 0, sumCount2 = 0;         // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;                   // absolute orientation
float a12_1, a22_1, a31_1, a32_1, a33_1;            // rotation matrix coefficients for Euler angles and gravity components
float a12_2, a22_2, a31_2, a32_2, a33_2;            // rotation matrix coefficients for Euler angles and gravity components
float deltat1 = 0.0f, sum1 = 0.0f, deltat2 = 0.0f, sum2 = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, lastUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                         // used to calculate integration interval

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1; // variables to hold latest sensor data values 
float ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2; // variables to hold latest sensor data values 
float lin_ax1, lin_ay1, lin_az1;             // linear acceleration (acceleration with gravity component subtracted)
float q1[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt1[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float lin_ax2, lin_ay2, lin_az2;             // linear acceleration (acceleration with gravity component subtracted)
float q2[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt2[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

MPU9250 MPU9250(intPin1); // instantiate MPU9250 class

void setup()
{
  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(1000);

  MPU9250.I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71 
  
  // Set up the interrupt pins, they're set as active high, push-pull
  pinMode(intPin1, INPUT);
  digitalWrite(intPin1, LOW);
  pinMode(intPin2, INPUT);
  digitalWrite(intPin2, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  delay(1000);

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axis motion sensor...");
  uint8_t c = MPU9250.getMPU9250ID(MPU1);
  Serial.print("MPU9250_1 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  uint8_t d = MPU9250.getMPU9250ID(MPU2);
  Serial.print("MPU9250_2 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  delay(1000);

if (c == 0x71 && d == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
{  
    Serial.println("MPU9250s 1 and 2 are online...");

    MPU9250.resetMPU9250(MPU1); // start by resetting MPU9250_1
    MPU9250.resetMPU9250(MPU2); // start by resetting MPU9250_2
    
    MPU9250.SelfTest(MPU1, SelfTest); // Start by performing self test and reporting values
    Serial.println("Self Test for MPU9250 #1:");
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

    MPU9250.SelfTest(MPU2, SelfTest); // Start by performing self test and reporting values
    Serial.println("Self Test for MPU9250 #2:");
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
    delay(1000); 

    // get sensor resolutions, only need to do this once, same for both MPU9250s for now
    aRes = MPU9250.getAres(Ascale);
    gRes = MPU9250.getGres(Gscale);
    mRes = MPU9250.getMres(Mscale);
    
   // Comment out if using pre-measured, pre-stored offset biases
    MPU9250.calibrateMPU9250(MPU1, gyroBias1, accelBias1); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("MPU1 accel biases (mg)"); Serial.println(1000.*accelBias1[0]); Serial.println(1000.*accelBias1[1]); Serial.println(1000.*accelBias1[2]);
    Serial.println("MPU1 gyro biases (dps)"); Serial.println(gyroBias1[0]); Serial.println(gyroBias1[1]); Serial.println(gyroBias1[2]);
    MPU9250.calibrateMPU9250(MPU2, gyroBias2, accelBias2); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println(" ");
    Serial.println("MPU2 accel biases (mg)"); Serial.println(1000.*accelBias2[0]); Serial.println(1000.*accelBias2[1]); Serial.println(1000.*accelBias2[2]);
    Serial.println("MPU2 gyro biases (dps)"); Serial.println(gyroBias2[0]); Serial.println(gyroBias2[1]); Serial.println(gyroBias2[2]);
    delay(1000); 
  
    MPU9250.initMPU9250(MPU1, Ascale, Gscale, sampleRate); 
    MPU9250.initMPU9250(MPU2, Ascale, Gscale, sampleRate); 
    Serial.println("MPU9250s 1 and 2 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      
   // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte e = MPU9250.getAK8963CID(MPU1);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 1 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    byte f = MPU9250.getAK8963CID(MPU2);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 2 "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    delay(1000); 
  
    // Get magnetometer calibration from AK8963 ROM
    MPU9250.initAK8963Slave(MPU1, Mscale, Mmode, magCalibration1); Serial.println("AK8963 1 initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
    Serial.println("Calibration values for mag 1: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration1[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration1[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration1[2], 2);
    MPU9250.initAK8963Slave(MPU2, Mscale, Mmode, magCalibration2); Serial.println("AK8963 2 initialized for active data mode...."); // Initialize device 2 for active mode read of magnetometer
    Serial.println("Calibration values for mag 2: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration2[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration2[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration2[2], 2);

   // Comment out if using pre-measured, pre-stored offset biases
    MPU9250.magcalMPU9250(MPU1, magBias1, magScale1);
    Serial.println("AK8963 1 mag biases (mG)"); Serial.println(magBias1[0]); Serial.println(magBias1[1]); Serial.println(magBias1[2]); 
    Serial.println("AK8963 1 mag scale (mG)"); Serial.println(magScale1[0]); Serial.println(magScale1[1]); Serial.println(magScale1[2]); 
    MPU9250.magcalMPU9250(MPU2, magBias2, magScale2);
    Serial.println("AK8963 2 mag biases (mG)"); Serial.println(magBias2[0]); Serial.println(magBias2[1]); Serial.println(magBias2[2]); 
    Serial.println("AK8963 2 mag scale (mG)"); Serial.println(magScale2[0]); Serial.println(magScale2[1]); Serial.println(magScale2[2]); 
    delay(2000); // add delay to see results before serial spew of data

    attachInterrupt(intPin1, myinthandler1, RISING);  // define interrupt for intPin output of MPU9250 1
    attachInterrupt(intPin2, myinthandler2, RISING);  // define interrupt for intPin output of MPU9250 2
  }
  else if ( c == 0x71 ) // d offline
  {
    Serial.print("Could not connect to MPU9250 2: 0x"); Serial.println(d, HEX);
    //while(1) ; // Loop forever if communication doesn't happen
  } else if ( d == 0x71 ) // c offline
  {
    Serial.print("Could not connect to MPU9250 1: 0x"); Serial.println(c, HEX);
  }
}

void loop()
{  
// If intPin1 goes high, either all data registers have new data
   if(intFlag1 == true) {   // On interrupt, read data
      intFlag1 = false;     // reset newData flag
      
     MPU9250.readMPU9250Data(MPU1, MPU9250Data1); // INT cleared on any read
   
    // Now we'll calculate the accleration value into actual g's
     ax1 = (float)MPU9250Data1[0]*aRes - accelBias1[0];  // get actual g value, this depends on scale being set
     ay1 = (float)MPU9250Data1[1]*aRes - accelBias1[1];   
     az1 = (float)MPU9250Data1[2]*aRes - accelBias1[2];  

    // Calculate the gyro value into actual degrees per second
     gx1 = (float)MPU9250Data1[4]*gRes;  // get actual gyro value, this depends on scale being set
     gy1 = (float)MPU9250Data1[5]*gRes;  
     gz1 = (float)MPU9250Data1[6]*gRes; 
  
//    if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
      MPU9250.readMagData(MPU1, magCount1);  // Read the x/y/z adc values
  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
      mx1 = (float)magCount1[0]*mRes*magCalibration1[0] - magBias1[0];  // get actual magnetometer value, this depends on scale being set
      my1 = (float)magCount1[1]*mRes*magCalibration1[1] - magBias1[1];  
      mz1 = (float)magCount1[2]*mRes*magCalibration1[2] - magBias1[2];  
      mx1 *= magScale1[0];
      my1 *= magScale1[1];
      mz1 *= magScale1[2]; 
//    }
   
    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now1 = micros();
    deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate1 = Now1;

    sum1 += deltat1; // sum for averaging filter update rate
    sumCount1++;

    MadgwickQuaternionUpdate1(-ax1, +ay1, +az1, gx1*pi/180.0f, -gy1*pi/180.0f, -gz1*pi/180.0f,  my1,  -mx1, mz1);
    }

    /* end of MPU9250 1 interrupt handling */
   }


      // If intPin2 goes high, either all data registers have new data
   if(intFlag2 == true) {   // On interrupt, read data
      intFlag2 = false;     // reset newData flag
      
     MPU9250.readMPU9250Data(MPU2, MPU9250Data2); // INT cleared on any read
   
    // Now we'll calculate the accleration value into actual g's
     ax2 = (float)MPU9250Data2[0]*aRes - accelBias2[0];  // get actual g value, this depends on scale being set
     ay2 = (float)MPU9250Data2[1]*aRes - accelBias2[1];   
     az2 = (float)MPU9250Data2[2]*aRes - accelBias2[2];  

    // Calculate the gyro value into actual degrees per second
     gx2 = (float)MPU9250Data2[4]*gRes;  // get actual gyro value, this depends on scale being set
     gy2 = (float)MPU9250Data2[5]*gRes;  
     gz2 = (float)MPU9250Data2[6]*gRes; 
  
//    if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
      MPU9250.readMagData(MPU2, magCount2);  // Read the x/y/z adc values
  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
      mx2 = (float)magCount2[0]*mRes*magCalibration2[0] - magBias2[0];  // get actual magnetometer value, this depends on scale being set
      my2 = (float)magCount2[1]*mRes*magCalibration2[1] - magBias2[1];  
      mz2 = (float)magCount2[2]*mRes*magCalibration2[2] - magBias2[2];  
      mx2 *= magScale2[0];
      my2 *= magScale2[1];
      mz2 *= magScale2[2]; 
//    }
   
  
    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now2 = micros();
    deltat2 = ((Now2 - lastUpdate2)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate2 = Now2;

    sum2 += deltat2; // sum for averaging filter update rate
    sumCount2++;

    MadgwickQuaternionUpdate2(-ax2, +ay2, +az2, gx2*pi/180.0f, -gy2*pi/180.0f, -gz2*pi/180.0f,  my2,  -mx2, mz2);
    
    /* end of MPU9250 2 interrupt handling */
   }
  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
//  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
//
//
//    if (!AHRS) {
//    delt_t = millis() - count;
//    if(delt_t > 500) {

   if(SerialDebug) {
//    Serial.println(" ");
//    Serial.print("ax1 = "); Serial.print((int)1000*ax1);  
//    Serial.print(" ay1 = "); Serial.print((int)1000*ay1); 
//    Serial.print(" az1 = "); Serial.print((int)1000*az1); Serial.println(" mg");
//    Serial.print("gx1 = "); Serial.print( gx1, 2); 
//    Serial.print(" gy1 = "); Serial.print( gy1, 2); 
//    Serial.print(" gz1 = "); Serial.print( gz1, 2); Serial.println(" deg/s");
//    Serial.print("mx1 = "); Serial.print( (int)mx1 ); 
//    Serial.print(" my1 = "); Serial.print( (int)my1 ); 
//    Serial.print(" mz1 = "); Serial.print( (int)mz1 ); Serial.println(" mG");
//    Serial.println(" ");
//    Serial.print("ax2 = "); Serial.print((int)1000*ax2);  
//    Serial.print(" ay2 = "); Serial.print((int)1000*ay2); 
//    Serial.print(" az2 = "); Serial.print((int)1000*az2); Serial.println(" mg");
//    Serial.print("gx2 = "); Serial.print( gx2, 2); 
//    Serial.print(" gy2 = "); Serial.print( gy2, 2); 
//    Serial.print(" gz2 = "); Serial.print( gz2, 2); Serial.println(" deg/s");
//    Serial.print("mx2 = "); Serial.print( (int)mx2 ); 
//    Serial.print(" my2 = "); Serial.print( (int)my2 ); 
//    Serial.print(" mz2 = "); Serial.print( (int)mz2 ); Serial.println(" mG");
    
//    Serial.println("MPU9250 1");
//    Serial.print("q0 = "); Serial.print(q1[0]);
//    Serial.print(" qx = "); Serial.print(q1[1]); 
//    Serial.print(" qy = "); Serial.print(q1[2]); 
//    Serial.print(" qz = "); Serial.println(q1[3]); 
//
//    Serial.println("MPU9250 2");
//    Serial.print("q0 = "); Serial.print(q2[0]);
//    Serial.print(" qx = "); Serial.print(q2[1]); 
//    Serial.print(" qy = "); Serial.print(q2[2]); 
//    Serial.print(" qz = "); Serial.println(q2[3]); 

    temperature1 = ((float) MPU9250Data1[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
    temperature2 = ((float) MPU9250Data2[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
//    Serial.print("Gyro 1 temperature is ");  Serial.print(temperature1, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
//    Serial.print("Gyro 2 temperature is ");  Serial.print(temperature2, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }               
    
    a12_1 =   2.0f * (q1[1] * q1[2] + q1[0] * q1[3]);
    a22_1 =   q1[0] * q1[0] + q1[1] * q1[1] - q1[2] * q1[2] - q1[3] * q1[3];
    a31_1 =   2.0f * (q1[0] * q1[1] + q1[2] * q1[3]);
    a32_1 =   2.0f * (q1[1] * q1[3] - q1[0] * q1[2]);
    a33_1 =   q1[0] * q1[0] - q1[1] * q1[1] - q1[2] * q1[2] + q1[3] * q1[3];
    pitch1 = -asinf(a32_1);
    roll1  = atan2f(a31_1, a33_1);
    yaw1   = atan2f(a12_1, a22_1);
    pitch1 *= 180.0f / pi;
    yaw1   *= 180.0f / pi; 
    yaw1   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw1 < 0) yaw1   += 360.0f; // Ensure yaw stays between 0 and 360
    roll1  *= 180.0f / pi;
    lin_ax1 = ax1 + a31_1;
    lin_ay1 = ay1 + a32_1;
    lin_az1 = az1 - a33_1;

    if(SerialDebug) {
    Serial.print("MPU9250 1 Yaw, Pitch, Roll: ");
    Serial.print(yaw1, 2);
    Serial.print(", ");
    Serial.print(pitch1, 2);
    Serial.print(", ");
    Serial.println(roll1, 2);

//    Serial.print("Grav_x, Grav_y, Grav_z: ");
//    Serial.print(-a31_1*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(-a32_1*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(a33_1*1000.0f, 2);  Serial.println(" mg");
//    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
//    Serial.print(lin_ax1*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(lin_ay1*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(lin_az1*1000.0f, 2);  Serial.println(" mg");
    }

    a12_2 =   2.0f * (q2[1] * q2[2] + q2[0] * q2[3]);
    a22_2 =   q2[0] * q2[0] + q2[1] * q2[1] - q2[2] * q2[2] - q2[3] * q2[3];
    a31_2 =   2.0f * (q2[0] * q2[1] + q2[2] * q2[3]);
    a32_2 =   2.0f * (q2[1] * q2[3] - q2[0] * q2[2]);
    a33_2 =   q2[0] * q2[0] - q2[1] * q2[1] - q2[2] * q2[2] + q2[3] * q2[3];
    pitch2 = -asinf(a32_2);
    roll2  = atan2f(a31_2, a33_2);
    yaw2   = atan2f(a12_2, a22_2);
    pitch2 *= 180.0f / pi;
    yaw2   *= 180.0f / pi; 
    yaw2   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw2 < 0) yaw2   += 360.0f; // Ensure yaw stays between 0 and 360
    roll2  *= 180.0f / pi;
    lin_ax2 = ax2 + a31_2;
    lin_ay2 = ay2 + a32_2;
    lin_az2 = az2 - a33_2;

    if(SerialDebug) {
    Serial.print("MPU9250 2 Yaw, Pitch, Roll: ");
    Serial.print(yaw2, 2);
    Serial.print(", ");
    Serial.print(pitch2, 2);
    Serial.print(", ");
    Serial.println(roll2, 2);

//    Serial.print("Grav_x, Grav_y, Grav_z: ");
//    Serial.print(-a31_2*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(-a32_2*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(a33_2*1000.0f, 2);  Serial.println(" mg");
//    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
//    Serial.print(lin_ax2*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(lin_ay2*1000.0f, 2);
//    Serial.print(", ");
//    Serial.print(lin_az2*1000.0f, 2);  Serial.println(" mg");
    
    Serial.print("rate 1 = "); Serial.print((float)sumCount1/sum1, 2); Serial.println(" Hz");
    sumCount1 = 0;
    sum1 = 0;    
    Serial.print("rate 2 = "); Serial.print((float)sumCount2/sum2, 2); Serial.println(" Hz");
    sumCount2 = 0;
    sum2 = 0;    
    }

    //Output for logging to spreadsheet
//    Serial.print(millis()); Serial.print(", ");
//    Serial.print(yaw1, 2); Serial.print(", "); Serial.print(pitch1, 2); Serial.print(", "); Serial.print(roll1, 2); Serial.print(", ");
//    Serial.print(yaw2, 2); Serial.print(", "); Serial.print(pitch2, 2); Serial.print(", "); Serial.println(roll2, 2);
    }
 }
    
//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void myinthandler1()
{
  intFlag1 = true;
}

void myinthandler2()
{
  intFlag2 = true;
}

void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


//void readAccelData(int16_t * destination)
//{
//  uint8_t rawData[6];  // x/y/z accel register data stored here
//  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
//  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
//  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
//  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
//}
//
//
//void readGyroData(int16_t * destination)
//{
//  uint8_t rawData[6];  // x/y/z gyro register data stored here
//  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
//  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
//  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
//  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
//}
//
//void readMagData(int16_t * destination)
//{
//  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
//  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
//  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
//  uint8_t c = rawData[6]; // End data read by reading ST2 register
//    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
//    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
//    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
//    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
//   }
//  }
//}

//int16_t readTempData()
//{
//  uint8_t rawData[2];  // x/y/z gyro register data stored here
//  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
//  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
//}
       
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

//void initMPU9250()
//{  
// // wake up device
//  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
//  delay(100); // Wait for all registers to reset 
//
// // get stable time source
//  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
//  delay(200); 
//  
// // Configure Gyro and Thermometer
// // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
// // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
// // be higher than 1 / 0.0059 = 170 Hz
// // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
// // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
//  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  
//
// // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
//  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
//                                    // determined inset in CONFIG above
// 
// // Set gyroscope full scale range
// // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
//  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
// // c = c & ~0xE0; // Clear self-test bits [7:5] 
//  c = c & ~0x03; // Clear Fchoice bits [1:0] 
//  c = c & ~0x18; // Clear GFS bits [4:3]
//  c = c | Gscale << 3; // Set full scale range for the gyro
// // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
//  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
//  
// // Set accelerometer full-scale range configuration
//  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
// // c = c & ~0xE0; // Clear self-test bits [7:5] 
//  c = c & ~0x18;  // Clear AFS bits [4:3]
//  c = c | Ascale << 3; // Set full scale range for the accelerometer 
//  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
//
// // Set accelerometer sample rate configuration
// // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
// // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
//  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
//  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
//  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
//  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
// // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
// // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
//
//  // Configure Interrupts and Bypass Enable
//  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
//  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
//  // can join the I2C bus and all can be controlled by the Arduino as master
//   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
//   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
//   delay(100);
//}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
//void calibrateMPU9250(float * dest1, float * dest2)
//{  
//  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
//  uint16_t ii, packet_count, fifo_count;
//  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
//  
// // reset device
//  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
//  delay(100);
//   
// // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
// // else use the internal oscillator, bits 2:0 = 001
//  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
//  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
//  delay(200);                                    
//
//// Configure device for bias calculation
//  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
//  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
//  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
//  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
//  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
//  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
//  delay(15);
//  
//// Configure MPU6050 gyro and accelerometer for bias calculation
//  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
//  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
//  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
//  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
// 
//  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
//  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
//
//    // Configure FIFO to capture accelerometer and gyro data for bias calculation
//  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
//  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
//  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
//
//// At end of sample accumulation, turn off FIFO sensor read
//  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
//  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
//  fifo_count = ((uint16_t)data[0] << 8) | data[1];
//  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
//  
//  for (ii = 0; ii < packet_count; ii++) {
//    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
//    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
//    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
//    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
//    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
//    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
//    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
//    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
//    
//    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
//    accel_bias[1] += (int32_t) accel_temp[1];
//    accel_bias[2] += (int32_t) accel_temp[2];
//    gyro_bias[0]  += (int32_t) gyro_temp[0];
//    gyro_bias[1]  += (int32_t) gyro_temp[1];
//    gyro_bias[2]  += (int32_t) gyro_temp[2];
//            
//}
//    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
//    accel_bias[1] /= (int32_t) packet_count;
//    accel_bias[2] /= (int32_t) packet_count;
//    gyro_bias[0]  /= (int32_t) packet_count;
//    gyro_bias[1]  /= (int32_t) packet_count;
//    gyro_bias[2]  /= (int32_t) packet_count;
//    
//  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
//  else {accel_bias[2] += (int32_t) accelsensitivity;}
//   
//// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
//  data[3] = (-gyro_bias[1]/4)       & 0xFF;
//  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
//  data[5] = (-gyro_bias[2]/4)       & 0xFF;
//  
//// Push gyro biases to hardware registers
//  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
//  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
//  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
//  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
//  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
//  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
//  
//// Output scaled gyro biases for display in the main program
//  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
//  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
//  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
//
//// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
//// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
//// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
//// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
//// the accelerometer biases calculated above must be divided by 8.
//
//  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//  
//  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
//  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
//  
//  for(ii = 0; ii < 3; ii++) {
//    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
//  }
//  
//  // Construct total accelerometer bias, including calculated average accelerometer bias from above
//  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//  accel_bias_reg[1] -= (accel_bias[1]/8);
//  accel_bias_reg[2] -= (accel_bias[2]/8);
//  
//  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//  data[1] = (accel_bias_reg[0])      & 0xFF;
//  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//  data[3] = (accel_bias_reg[1])      & 0xFF;
//  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//  data[5] = (accel_bias_reg[2])      & 0xFF;
//  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
// 
//// Apparently this is not working for the acceleration biases in the MPU-9250
//// Are we handling the temperature correction bit properly?
//// Push accelerometer biases to hardware registers
//  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
//
//// Output scaled accelerometer biases for display in the main program
//   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
//   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
//   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
//}

//void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
//{
//   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
//   uint8_t selfTest[6];
//   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
//   float factoryTrim[6];
//   uint8_t FS = 0;
//   
//  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
//  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
//  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
//  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
//  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g
//
//  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
//  
//  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
//  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
//  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
//  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
//  
//    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
//  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
//  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
//  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
//  }
//  
//  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
//  aAvg[ii] /= 200;
//  gAvg[ii] /= 200;
//  }
//  
//// Configure the accelerometer for self-test
//   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
//   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
//   delay(25);  // Delay a while to let the device stabilize
//
//  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
//  
//  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
//  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
//  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
//  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
//  
//    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
//  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
//  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
//  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
//  }
//  
//  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
//  aSTAvg[ii] /= 200;
//  gSTAvg[ii] /= 200;
//  }   
//  
// // Configure the gyro and accelerometer for normal operation
//   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
//   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
//   delay(25);  // Delay a while to let the device stabilize
//   
//   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
//   SelfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
//   SelfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
//   SelfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
//   SelfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
//   SelfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
//   SelfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results
//
//  // Retrieve factory self-test value from self-test code reads
//   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
//   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
//   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
//   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
//   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
//   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
// 
// // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
// // To get percent, must multiply by 100
//   for (int i = 0; i < 3; i++) {
//     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
//     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
//   }
//   
//}

// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
