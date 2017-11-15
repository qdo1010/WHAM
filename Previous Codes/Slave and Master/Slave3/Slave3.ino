#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include "LSM9DS0.h"
#include <SoftwareSerial.h>
///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
const byte INT1XM = 0; // INT1XM tells us when accel data is ready
const byte INT2XM = 1; // INT2XM tells us when mag data is ready
const byte DRDYG  = 4; // DRDYG  tells us when gyro data is ready

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

float pitch, yaw, roll, heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint16_t lastUpdate = 0;    // used to calculate integration interval
uint16_t now = 0;           // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
uint8_t masterReceive[3] = {0x00,0x00,0x00} ;
String valueString = "a"; 
byte valAll[1]; 
boolean startUp = true; 
int incomingByte = 0;
byte val1; 
byte val2;  
int deviceType = 1;
int deviceID = 0; 
bool blinkState = false;


#define LED_BLUE       7
SoftwareSerial p2pSerial(9,6); //RX, TX


void setup()
{
  Serial.begin(115200); // Start serial at 115200 bps
 
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG,  INPUT);
            
  // begin() returns a 16-bit value which includes both the gyro 
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.
  uint16_t status = dof.begin();
 
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
  
  // Set data output ranges; choose lowest ranges for maximum resolution
  // Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
  dof.setAccelScale(dof.A_SCALE_2G);
  // Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
  dof.setGyroScale(dof.G_SCALE_245DPS);
  // Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
  dof.setMagScale(dof.M_SCALE_2GS);
  
  // Set output data rates  
  // Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50, 
  //                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)
  dof.setAccelODR(dof.A_ODR_100); // Set accelerometer update rate at 100 Hz
    
  // Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
  // Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
  // Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
  // to get a smooth ~150 Hz filter update rate
  dof.setAccelABW(dof.A_ABW_50); // Choose lowest filter setting for low noise
 
  // Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
  //                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
  dof.setGyroODR(dof.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

  // Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
  dof.setMagODR(dof.M_ODR_125); // Set magnetometer to update every 80 ms
  pinMode(LED_BLUE, OUTPUT);   
    //pinMode(LED_RED, OUTPUT); 
    digitalWrite(LED_BLUE, HIGH); 
    //Serial.begin(9600);
    p2pSerial.begin(9600);
    delay(50); 
  
    //Wait until data from peer to peer network
  /*  while(true){ 
      
      if(p2pSerial.available()>1) { 
   
        if(p2pSerial.read()==200) { 
          incomingByte = p2pSerial.read();
          //assign address from incoming data  
          deviceID=incomingByte; 
          //Serial.print(deviceID);
          //send data to the next device
          p2pSerial.write(200);
          p2pSerial.write(incomingByte+1);
          digitalWrite(LED_BLUE, LOW);  
          break; 
        }
      }
    }//end while
    p2pSerial.end(); 
    Wire.begin(deviceID); // Start I2C Bus as a Slave (Device Number 2)
    Wire.onRequest(requestEvent); // register event to send data to master (respond to requests)
    Wire.onReceive(receiveEvent); //register even to receice data from master
*/
}

void loop()
{
  if(digitalRead(DRDYG)) {  // When new gyro data is ready
  dof.readGyro();           // Read raw gyro data
    gx = dof.calcGyro(dof.gx);   // Convert to degrees per seconds
    gy = dof.calcGyro(dof.gy);
    gz = dof.calcGyro(dof.gz);
  }
  
  if(digitalRead(INT1XM)) {  // When new accelerometer data is ready
    dof.readAccel();         // Read raw accelerometer data
    ax = dof.calcAccel(dof.ax);   // Convert to g's
    ay = dof.calcAccel(dof.ay);
    az = dof.calcAccel(dof.az);
  }
  
  if(digitalRead(INT2XM)) {  // When new magnetometer data is ready
    dof.readMag();           // Read raw magnetometer data
    mx = dof.calcMag(dof.mx);     // Convert to Gauss
    my = dof.calcMag(dof.my);
    mz = dof.calcMag(dof.mz);
  }

  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;

  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination), 
  // looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI;
  roll  *= 180.0f / PI;
  
  // display quaternion values in InvenSense Teapot demo format:
  teapotPacket[2] = q[0];
  teapotPacket[3] = q[0];
  teapotPacket[4] = q[1];
  teapotPacket[5] = q[1];
  teapotPacket[6] = q[2];
  teapotPacket[7] = q[2];
  teapotPacket[8] = q[3];
  teapotPacket[9] = q[3];
  //Serial.write(teapotPacket, 14);
  teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
  
  Serial.print(yaw, 2);
  Serial.print(", ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);

  // With ODR settings of 400 Hz, 380 Hz, and 25 Hz for the accelerometer, gyro, and magnetometer, respectively,
  // the filter is updating at a ~125 Hz rate using the Madgwick scheme and ~165 Hz using the Mahony scheme 
  // even though the display refreshes at only 2 Hz.
  // The filter update rate can be increased by reducing the rate of data reading. The optimal implementation is
  // one which balances the competing rates so they are about the same; that is, the filter updates the sensor orientation
  // at about the same rate the data is changing. Of course, other implementations are possible. One might consider
  // updating the filter at twice the average new data rate to allow for finite filter convergence times.
  // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
  // the processor speed (8 MHz for the 3.3V Pro Mini), and the sensor ODRs, especially the magnetometer ODR:
  // smaller ODRs for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
  // filter update rates of ~110 and ~135 Hz for the Madgwick and Mahony schemes, respectively. 
  // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
  // With low ODR settings of 100 Hz, 95 Hz, and 6.25 Hz for the accelerometer, gyro, and magnetometer, respectively,
  // the filter is updating at a ~150 Hz rate using the Madgwick scheme and ~200 Hz using the Mahony scheme.
  // These filter update rates should be fast enough to maintain accurate platform orientation for 
  // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
  // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
  // The 3.3 V 8 MHz Pro Mini is doing pretty well!

}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}


//This function assembles a reply to a master's request
void requestEvent()
{
  //Wire.write("abcdefg",14);
    //The communication sends the number of sensors and type of device. 
    if (startUp) { 
        byte a = deviceID & 0xFF; //Use this format to send 16bit integer to java. 
        byte b = (deviceID >>8 ) & 0xFF;
        
       // byte c = numberOfSensors & 0xFF; ; 
       // byte d = (numberOfSensors >>8 ) & 0xFF;
        
       // byte e = deviceType & 0xFF; ; 
       // byte f = (deviceType >>8 ) & 0xFF;
        
       // int tempread = digitalRead(cut1pin); 
       // byte g = tempread & 0xFF; ; 
       // byte h = (tempread >>8 ) & 0xFF;     
        
      //  tempread = digitalRead(cut2pin); 
      //  byte i = tempread & 0xFF; ; 
      //  byte j = (tempread >>8 ) & 0xFF;   
        
      //  tempread = digitalRead(cut2pin); 
      //  byte k = tempread & 0xFF; ; 
       // byte l = (tempread >>8 ) & 0xFF; 
        
       // tempread = digitalRead(cut2pin); 
      //  byte m = tempread & 0xFF; ; 
      //  byte n = (tempread >>8 ) & 0xFF; 
        
        //byte All [] = {a,b,c,d,e,f,g,h,i,j,k,l,m,n};  
        byte All [] = {a,b};  
        Wire.write(All, 2); // respond with message 
       // Wire.write("0000000000000",14);
        startUp = false;
    } 

    //Otterwise, send data from the on-board sensors. 
    else { 
        //digitalWrite(LED_RED, HIGH);
      //  lightAnalogRead = analogRead(ADC_LIGHT_PIN); // Read light value 
      //  thermistorAnalogRead = lightAnalogRead; //Read thermistor valie
        //positionAnalogRead = analogRead(A1); //Read position
        
        byte a = deviceID & 0xFF;
        byte b = (deviceID >>8 ) & 0xFF;
        
     //   byte c = lightAnalogRead & 0xFF; ; 
      //  byte d = (lightAnalogRead >>8 ) & 0xFF;
     
     //   byte e = thermistorAnalogRead & 0xFF; ; 
     //   byte f = (thermistorAnalogRead >>8 ) & 0xFF;
        
    //    byte g = distanceReading & 0xFF;
      //  byte h = (0x00 >>8 ) & 0xFF;
       
   /*     byte All [] = {a,b,c,d,e,f,g,h, 
                      fifoBuffer[1], fifoBuffer[0], 
                      fifoBuffer[5], fifoBuffer[4], 
                      fifoBuffer[9], fifoBuffer[8],
                      fifoBuffer[13], fifoBuffer[12]};     
*/
       byte All [] = {a,b, 
                      teapotPacket[3], teapotPacket[2], 
                      teapotPacket[5], teapotPacket[4], 
                      teapotPacket[7], teapotPacket[6],
                      teapotPacket[9], teapotPacket[8]};
        byte *mypointer; 
        mypointer = All;
        byte crc8 = CRC8(mypointer, 10);
        
/*        byte data [] = {a,b,c,d,e,f,g,h, 
              fifoBuffer[1], fifoBuffer[0], 
              fifoBuffer[5], fifoBuffer[4], 
              fifoBuffer[9], fifoBuffer[8],
              fifoBuffer[13], fifoBuffer[12],
              crc8, 0x00}; */
        byte data [] = {a,b, 
              teapotPacket[3], teapotPacket[2], 
              teapotPacket[5], teapotPacket[4], 
              teapotPacket[7], teapotPacket[6],
              teapotPacket[9], teapotPacket[8],
              crc8, 0x00};          
        //  byte data[] =      {a,b, ax, ay, az, gx, gy, gz, mx, my, mz, crc8, 0x00};
       //byte data[] = {a,b, q[0], q[0], q[1], q[1], q[3], q[3], q[4],q[4], crc8, 0x00};            
        Wire.write(data, 12); 
        //digitalWrite(LED_RED, LOW);    
    }//end else        
}//end requestEvent

//This function sets led color based on master command
void receiveEvent(int howMany)
{
  if (Wire.available()) {    
      masterReceive[0] = Wire.read(); 
      masterReceive[1] = Wire.read(); 
      masterReceive[2] = Wire.read(); 
  }
//    pixels.setPixelColor(0, pixels.Color(masterReceive[0],masterReceive[1],masterReceive[2])); 
//    pixels.show();
}
/*
int readInternalTemperature() {
    int result;
    // Read temperature sensor against 1.1V reference
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;
    //result = (result - 125) * 1075;
    return result;
}*/

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//From here: 
///http://www.leonardomiliani.com/en/2013/un-semplice-crc8-per-arduino/
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

