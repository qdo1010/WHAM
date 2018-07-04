#include <Wire.h>
#include <SoftwareSerial.h>

#define P2P_SERIAL_RX 10
#define P2P_SERIAL_TX 11
//10 and 11 is for Arduino UNO, 9 and 6 should work w Flora
//#define P2P_SERIAL_RX 9
//#define P2P_SERIAL_TX 6

boolean lookUpPresence[128];
int lookUpNumberOfSensors[128];
int x=0;
unsigned int deviceID = 200;

SoftwareSerial p2pSerial(P2P_SERIAL_RX,P2P_SERIAL_TX);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Connected");
  Wire.begin();
  delay(10);
  p2pSerial.begin(9600);
  delay(90);
  int foo = 200;
  p2pSerial.write(foo);
  foo=1;
  p2pSerial.write(foo);
  delay(4000);

  findWhoIsAround();

}

void loop() {
  for(int i=0; i<128; i++) { 
    if(lookUpPresence[i]) { 
      if (i == 8 ) {
      requestFromDevice(i,8); //this is for the motion sensor 
    }
    else requestFromDevice(i,8);
    }
  }//end for

  if (Serial.available() >= 1) { 
    delay(5);  //The communications sometimes crashes without this delay. 
    int whereToSend = Serial.read() *256; 
    whereToSend = whereToSend + Serial.read(); 
    byte command1 = Serial.read();
    byte command2 = Serial.read(); 
    byte command3 = Serial.read(); 
    delay(15); //The communications might crash without this delay 
    //Check just in case if the commands are below 128. This is the maximum that can be sent by java Char (-127 to 127) 
    if (command1 <128 && command2<128 & command3<128) { 
    sendByteToSlave(whereToSend, command1,command2,command3);
    }
    Serial.flush();
}
 
}
void setToZeros() { 
  unsigned int deviceID = 200 ;
}//end setToZeros


void findWhoIsAround()
{
  for (int i=0; i<128; i++)
  {
    requestFromDeviceDuringStart(i,2);
    if(deviceID==i)
    {
       Serial.print(i);
       Serial.println(" is present");
       lookUpPresence[i] = true; 
      } 
      else {
        lookUpPresence[i] = false;
      } 
      //delay(1000);
  }//end for
}//end findwhoIsAround


void requestFromDevice(int device, int bytes){
    unsigned char inputArray[bytes];
    unsigned char dataArray[bytes-2];
    setToZeros(); 
    
   if (Wire.requestFrom(device, bytes)>0) {  
      while(Wire.available())    
      { 
        unsigned char c= Wire.read(); 
        Serial.println(c);
        inputArray[x] = c; 
        x++;   
      }//end while 
      
      x = 0; 
      
      for (int i=0; i<bytes-2; i++) { 
        dataArray[i] = inputArray[i];
      }
      
      byte *mypointer; 
      mypointer = dataArray;
      byte crc8 = CRC8(mypointer, 6);
      byte crc8in = inputArray[bytes-2];
      
      if(crc8 == crc8in) { 
        inputArray[bytes-2]= 0x02;
        inputArray[bytes-1]= 0x00;
      }
      else{ 
        inputArray[bytes-2]= 0x01;
        inputArray[bytes-1]= 0x00;
      }
        
      
     
      //Send data to the PC immidiately. 
      Serial.print("S");
      Serial.print(",");
//      Serial.print(device);
//      Serial.print(",");
      for(int i=0; i<bytes; i = i+2) { 
       // Serial.print((inputArray[i+1]<<8) + inputArray[i]);
        Serial.print(inputArray[i] );
        if (i==bytes-2) { 
        //DO nothing
          }
        else
          Serial.print(",");
      }
      Serial.println(" ");
      deviceID = (inputArray[1]<<8) + inputArray[0]; 
      //Serial.println(deviceID);
            
     /*for(int i=0; i<bytes-2; i++) { 
       Serial.print(dataArray[i],HEX); 
       Serial.print(","); 
     }
     Serial.print(",");
      Serial.print(crc8);
      Serial.print(","); 
      Serial.print(crc8in);
      Serial.println(" ");*/
      
      
   }//end if 
 
 
     //Didnt find the device.
     else {
      Serial.print("S");
      Serial.print(",");
      Serial.print(device);
      Serial.print(",");
      for(int i=2; i<bytes; i = i+2) { 
        Serial.print(100);
        if (i==bytes-2) { 
        //DO nothing
          }
        else
          Serial.print(",");
      }
      Serial.println(" ");
      deviceID = device; 
      //Serial.println(deviceID); 
      
      
     
   }
}


void requestFromDeviceDuringStart(int device, int bytes) {
  unsigned char inputArray[bytes] ;
    setToZeros(); 
    if (Wire.requestFrom(device, bytes)>0) { 
      while(Wire.available())    // slave may send less than requested
      { 
        unsigned char  c= Wire.read(); 
        inputArray[x] = c; 
        x++;   
      }
      x = 0; 

      //Send to the PC immidiately 
      Serial.print("A");
      Serial.print(",");
      Serial.print(device);
      Serial.print(",");
      for(int i=0; i<bytes; i = i+2) { 
        Serial.print((inputArray[i+1]<<8) + inputArray[i]);
        if (i==bytes-2) { 
        //DO nothing
          }
        else
          Serial.print(",");
      }
      Serial.println(" ");
      //deviceID = (inputArray[1]<<8) + inputArray[0];
      deviceID = device; //cause the previous one mess up the lookup table 
      delay(5); //was 5 before
    }
    else{
      //Serial.println("wtf");
    }
} //end requestFromDeviceDuringStart


void sendByteToSlave(int address, uint8_t command1, uint8_t command2, uint8_t command3) { 
  Wire.beginTransmission(address); 
  Wire.write(command1); 
  Wire.write(command2); 
  Wire.write(command3); 
  Wire.endTransmission(); 
}  //end sendByteToSlave


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


