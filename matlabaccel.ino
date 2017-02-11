#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//#define RTS  10
//#define RXI  11
//#define TXO  12
//#define CTS  13
//#define MODE -1

#define BUFSIZE 128
#define VERBOSE_MODE false


MPU6050 mpu; //init 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector




// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

#define OUTPUT_READABLE_YAWPITCHROLL
// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin


// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

int battery = 0;
int count = 0;

float x_velo = 0;
float y_velo = 0;
float z_velo = 0;
float x_currAccel;
float y_currAccel;
float z_currAccel;
float x_prevAccel;
float y_prevAccel;
float z_prevAccel;

unsigned long prevTime = millis();
unsigned long currTime;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;

}



void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
 while (!Serial); // required for Flora & Micro
  delay(500);
  
  Serial.begin(115200);
  
//  //Serial.println(F("BLE Starting"));
//   boolean success;
// // delay(5000);
//
//  if (! ble.begin(VERBOSE_MODE)) {
//    //Serial.println( F("FAILED!") );
//
//  }
//
//  //Serial.println( F("OK!") );
//
// // Serial.print(F("Factory reset: "));
//  if(! ble.factoryReset()) {
//  //  Serial.println(F("FAILED."));
////    while(1);
//  }
//
// // Serial.println( F("OK!") );
//
//  ble.echo(false);
//    Serial.println("Requesting Bluefruit info:");
//  /* Print Bluefruit information */
//  ble.info();
//
//  Serial.println(F("Set device name: "));
//  if(! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=VRM"))) {
//    Serial.println(F("FAILED."));
//  //  while(1);
//  }
//  
//  /* Add the Heart Rate Service definition */
//  /* Service ID should be 1 */
//  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
//  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
//  if (! success) {
//    error(F("Could not add HRM service"));
//  }
//
//  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
//  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
//    if (! success) {
//    error(F("Could not add HRM characteristic"));
//  }
//  Serial.println(F("OK!"));
//  /* Add the Body Sensor Location characteristic */
//  /* Chars ID for Body should be 2 */
//  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
//  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
//    if (! success) {
//    error(F("Could not add BSL characteristic"));
//  }
//
//  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
//  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
//  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );
//
//  ble.reset();
// 
//init gyro and accel
 
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 mpu.initialize();
//  Serial.println("Testing device connections...");
 //   Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

 // Serial.println(F("OK!"));

// Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();


    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(136);
    mpu.setYGyroOffset(20);
    mpu.setZGyroOffset(23);
    mpu.setXAccelOffset(-1892);
    mpu.setYAccelOffset(1101);
    mpu.setZAccelOffset(1590);
    
if (devStatus == 0) {
        // turn on the DMP, now that it's ready
  //      Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
    //    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //    Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


//  while (! ble.isConnected())
//    delay(500);

//  Serial.println(F("Connected"));
 //  interruptSetup();
//  batteryLevel();

}

void loop() {
   
  if (QS == true){ 
  int heart_rate = BPM;

//  Serial.print(F("Updating HRM value to "));
  //Serial.println(heart_rate);
 // Serial.println(F(" BPM"));

  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
//  ble.print( F("AT+GATTCHAR=") );
//  ble.print( hrmMeasureCharId );
//  ble.print( F(",00-") );
//  ble.println(heart_rate, HEX);
  
  /* Check if command executed OK */
//  if ( !ble.waitForOK() )
//  {
 //   Serial.println(F("Failed to get response!"));
 // }
  QS = false;
  /* Delay before next measurement update */
  }
 
   
  delay(5);
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
     
   // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
       mpu.dmpGetAccel(&aa, fifoBuffer);
       mpu.dmpGetQuaternion(&q, fifoBuffer);
       mpu.dmpGetGravity(&gravity, &q);
       mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
       mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  sensors_event_t event;
//  bno.getEvent(&event);

 // ble.print("AT+BLEUARTTX=");
 /* ble.print(ypr[0] * 180/M_PI, 1);
  ble.print(",");
  ble.print(ypr[1] * 180/M_PI, 1);
  ble.print(",");
  ble.print(ypr[2] * 180/M_PI, 1);
  ble.print(",");
  ble.print(aaReal.x);
  ble.print(",");
  ble.print(aaReal.y);
  ble.print(",");
  ble.print(aaReal.z);
  ble.print(",");
  ble.print(battery, DEC);
  ble.println("|");
  ble.readline(200);
*/

  currTime = millis();
  
  

  x_velo = x_velo + ((aaReal.x + x_prevAccel)/2)*(currTime - prevTime)/1000; //1000 added to get the same units
   y_velo = y_velo + ((aaReal.y + y_prevAccel)/2)*(currTime - prevTime)/1000; //1000 added to get the same units
    z_velo = z_velo + ((aaReal.z + z_prevAccel)/2)*(currTime - prevTime)/1000; //1000 added to get the same units

  x_prevAccel = aaReal.x;
  y_prevAccel = aaReal.y;
  z_prevAccel = aaReal.z;
  prevTime = currTime;
    
  Serial.print(ypr[0] * 180/M_PI, 1);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI, 1);
  Serial.print("\t");
  Serial.print(ypr[2] * 180/M_PI, 1);
  Serial.print("\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.print(aaReal.z);
  Serial.print("\t");
  Serial.print(x_velo);
  Serial.print("\t");
  Serial.print(y_velo);
  Serial.print("\t");
  Serial.print(z_velo);
  Serial.print("\n");
  
    }
  if(count == 5000) {
    batteryLevel();
    count = 0;
  } else {
    delay(200);
    count++;
  }

}

void batteryLevel() {

  ble.println("AT+HWVBAT");
  ble.readline(1000);

  if(strcmp(ble.buffer, "OK") == 0) {
    battery = 0;
  } else {
    battery = atoi(ble.buffer);
    ble.waitForOK();
  }
}
