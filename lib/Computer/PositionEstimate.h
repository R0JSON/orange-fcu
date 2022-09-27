#include<Arduino.h>

class PositionEstimate
{
    public:
        void integrateVelocityAlt(float &velocity, float deltaT, float altitude, float prevAltitude);
        void integrateVelocityAcc(float ax, float ay, float az, float pitch, float yaw, float roll, float deltaT, float &velocityX, float &velocityY, float &velocityZ);
        void integrateAltitudeVel(float vx, float vy, float vz, float deltaT, float pitch, float yaw, float roll, float &altitude);
};