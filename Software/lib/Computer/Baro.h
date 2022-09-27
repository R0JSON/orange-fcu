#include <Adafruit_BMP280.h>
#include "Queue.h"



class Barometer
{
    public:
        bool getAltitude(float &altitude, float& maxAltitude);
        bool initBaro(int altitudeAverageCnt,bool DEBUG_OUTPUT);

    private:
        void barometerCalibration();

   public:
     float groundPressure;
     Queue<float> prevAltitude = Queue<float>(10);
     int aAC;
};






 
