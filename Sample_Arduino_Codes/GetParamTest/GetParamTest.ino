/*
Interfacing test for hooking up an Arduino module with Robot Controller
Date:     12 Jan 2021
Author:   Fabian Kung
 
 The circuit: 
 * RX is digital pin 10 (connect to TX of other device, e.g. HC-05 TX)
 * TX is digital pin 11 (connect to RX of other device, e.g. HC-05 RX) 
 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // pin 10 = RX, pin 11 = TX

char   bytRXdata[12];
#define  _MAX_RX_DATA_LENGTH 12

void setup()  
{
  // Open the SoftwareSerial port and set the data rate.
  mySerial.begin(57600);
  // Open hardware serial port.
  Serial.begin(57600);
}

void loop() // run over and over
{
  int nCount;

  Serial.println("Start send command to robot controller");
  mySerial.write("GD000");      // Send 6 bytes. Command, Get, Distance sensor output.
  mySerial.write("\n");         // End with newline character.  
                                // Note: The Robot Controller will response with "DXXX"
                                // where XXX are 3 numeric characters that show the 
                                // distance of the object to the distance sensor. A 
                                // large value indicates the object is very near the
                                // distance sensor.
  nCount = 0;                   // Reset counter.
  delay(1);                     // 1 msec delay for the robot controller to reply.
  // Check if there is any byte available, get all
  // the data bytes from software serial.
  while (mySerial.available())  
  {
    bytRXdata[nCount] = mySerial.read();
    nCount++;
    if (nCount == _MAX_RX_DATA_LENGTH)  // If receive buffer is already full, terminate.
    {
      mySerial.flush();
      break;
    }
  }

  // Send the data received from software serial to hardware serial.
  if (nCount>0)
  {
    Serial.write(bytRXdata,nCount);
    Serial.write("\n");
  }

  delay(1000);
}
