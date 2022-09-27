#include "StateMachine.h"

void StateMachine::initStateMachine(float launchDetectionTreshold, float freefallAccelerationTreshold, float apogeeDetectionTreshold, float touchdownDetectionTreshold)
{
    lDT=launchDetectionTreshold;
    fAT=freefallAccelerationTreshold;
    aDT=apogeeDetectionTreshold;
    tDT=touchdownDetectionTreshold;
}

bool StateMachine::liftoffDetection(int state, float az, bool DEBUG_OUTPUT)
{
  if(state>=2)
  {
    return true;
  }
  
  if(az>=lDT*10.0f)
  {
    if(DEBUG_OUTPUT)
      Serial.println("Launch detected!");
    
    return true;
  }
  return false;

}  

bool StateMachine::apogeeDetection(int state, float ax, float ay, float az, float altitude, float maxAltitude)
{
  if(state>=3)
  {
    return true;
  }

  if(altitude+aDT<=maxAltitude && altitude>10.0f)
  {
    float at=sqrt(ax*ax+ay*ay+az*az);

    if(at<=fAT)
    {
      //return true;
    }
    return true;

  }

  return false;
}

bool StateMachine::touchdownDetection(int state, float altitude, float ax, float ay, float az)
{
    if(state>=8)
    {
        return true;
    }

    float at=sqrt(ax*ax+ay*ay+az*az);

    if(altitude<=tDT && state>=3 && at>=8.0f)
    {
        return true;
    }
    return false;
}
 