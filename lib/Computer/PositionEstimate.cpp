#include "PositionEstimate.h"

void PositionEstimate::integrateVelocityAlt(float &velocity, float deltaT, float altitude, float prevAltitude)
{
    velocity=(altitude-prevAltitude)/deltaT;
}

void PositionEstimate::integrateVelocityAcc(float ax, float ay, float az, float pitch, float yaw, float roll, float deltaT, float &velocityX, float &velocityY, float &velocityZ)
{
    velocityX+=ax*deltaT;
    velocityX+=ay*cos(roll)*deltaT;
    velocityX+=az*cos(yaw)*deltaT;
    
    velocityY+=ax*cos(pitch)*deltaT;
    velocityY+=ay*deltaT;
    velocityY+=az*cos(yaw)*deltaT;

    velocityZ+=ax*cos(pitch)*deltaT;
    velocityZ+=ay*cos(roll)*deltaT;
    velocityZ+=az*deltaT;

}

//to rework
void PositionEstimate::integrateAltitudeVel(float vx, float vy, float vz, float deltaT, float pitch, float yaw, float roll, float &altitude)
{
    altitude+=vx*cos(pitch)*deltaT;
    altitude+=vy*cos(roll)*deltaT;
    altitude+=vz*deltaT; 
}
