#include<Arduino.h>

class StateMachine
{
    public:
        void initStateMachine(float launchDetectionTreshold, float freefallAccelerationTreshold, float apogeeDetectionTreshold, float touchdownDetectionTreshold);
        bool liftoffDetection(int state, float az, bool DEBUG_OUTPUT);
        bool apogeeDetection(int state, float ax, float ay, float az, float altitude, float maxAltitude);
        bool touchdownDetection(int state, float altitude, float ax, float ay, float az);

    private:
        float lDT;
        float fAT;
        float aDT;
        float tDT;
};