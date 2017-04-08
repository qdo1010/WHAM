#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);  


}

String data = "";
void loop() {
  // put your main code here, to run repeatedly:
  Wire.requestFrom(8,6);
  data ="";
  while (Wire.available()){
    data += (char)Wire.read();
    
  }
  Serial.println(data);
  delay(500);
}


