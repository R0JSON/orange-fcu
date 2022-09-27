#include <ESP32_Servo.h>

//#include "Pid.h"


class servoControl
{
    public:
        void initServo();
        void stabilize(float servoValueZ, float servoValueX);
    
    private:
        float pitch,yaw,roll;
        float T,deltaT,prevT;

};
