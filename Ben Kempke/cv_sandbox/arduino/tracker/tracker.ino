#include <Servo.h>

Servo servo, servo_v;
int servoPosition = 90;
int servoPositionV = 90;

int incomingByte = 0;   // for incoming serial data

void setup()
{
  Serial.begin(9600); // // opens serial port, sets data rate to 9600 bps
  
  servo.attach(5); // attaches the servo on pin 5 to the servo object
  servo.write(servoPosition); // set the servo at the mid position
  
  servo_v.attach(6);
  servo_v.write(servoPositionV);
}

void loop()
{
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    switch(incomingByte)
    {
      // Rotate camera left
      case 'l':
      servoPosition+=1;
      
      if (servoPosition > 180)
      {
        servoPosition = 180;
      }

      break;
      
      // Rotate camera right
      case 'r':
      servoPosition-=1;
      
      if (servoPosition < 0)
      {
        servoPosition = 0;
      }

      break;
      
      case 'u':
      servoPositionV+=1;
      
      if(servoPositionV > 180)
        servoPositionV = 180;
      break;
      
      case 'd':
      servoPositionV-=1;
      if(servoPositionV < 0)
        servoPositionV = 0;
      break;
      
      // Center camera
      case 'c':
      servoPosition = 90;
      servoPositionV = 90;
      break;
    }
    
    servo.write(servoPosition);
    servo_v.write(servoPositionV);
  }
}
