/*
Interfacing test for hooking up an Arduino Uno/Nano with Robot Controller
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
  static int nSequence = 0;
  
  Serial.println("Start send command to robot controller");

  if (nSequence == 0)
  {
    nSequence++;
    mySerial.write("M0+45");    // Send 6 bytes. Command, turn Motor 0 to +45
                                // degrees.
    mySerial.write("\n");       // End with newline character.  
  }
  else
  {
    nSequence = 0;
    mySerial.write("M0-45");    // Send 6 bytes. Command, turn Motor 0 to -45
                                // degrees.
    mySerial.write("\n");       // End with newline character.      
  }
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

  delay(4000);
}
