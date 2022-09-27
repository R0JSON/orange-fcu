#include <ESP32_Servo.h>
#include <Arduino.h>
#include "ServoControl.h"
#include "Config.h"
Servo Xp; //X+
Servo Xm; //X-
Servo Zp; //Z+
Servo Zm; //Z-

void servoControl::initServo()
{
    //Attach servos to the given pins
    Xp.attach(servo_pin_0);
    Xm.attach(servo_pin_1);
    Zp.attach(servo_pin_2);
    Zm.attach(servo_pin_3);

    //Test sequence
    Xp.write(90);
    Xm.write(90);
    Zp.write(90);   
    Zm.write(90); 

    delay(2000); 

    Xp.write(45);
    Xm.write(45);
    Zp.write(45);   
    Zm.write(45); 

    delay(2000);

    Xp.write(135);
    Xm.write(135);
    Zp.write(135);   
    Zm.write(135); 
   
    delay(2000);
}


void servoControl::stabilize(float servoValueZ, float servoValueX)
{
    
    
    
    Xp.write(90-servoValueX);
    Xm.write(90+servoValueX);
    Zp.write(90-servoValueZ);
    Zm.write(90+servoValueZ);
    
}
