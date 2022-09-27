#include "Baro.h"

Adafruit_BMP280 bmp;

bool Barometer::getAltitude(float &altitude, float &maxAltitude)
{
    float top,sum=0,alt;
    
    alt=bmp.readAltitude(groundPressure/100.0f);

    if(isnan(alt))
    {
        return false;
    }

    prevAltitude.push(alt);
  
     //Rolling average of altitude
     top=prevAltitude.peek();
     prevAltitude.pop();
     sum+=top;

    for(int i=0;i<aAC-1;i++)
    {
        top=prevAltitude.peek();
        prevAltitude.pop();
        sum+=top;
        prevAltitude.push(top);
    }
  
    altitude=sum/aAC;
    //altitude=alt;
    maxAltitude=max(maxAltitude,altitude);

    return true;

} 


void Barometer::barometerCalibration()
{
    float reading;
    groundPressure=bmp.readPressure();

    for(int i=0;i<aAC;i++)
    {
      reading=bmp.readAltitude(groundPressure/100.0f);
      prevAltitude.push(reading);
    }
}

bool Barometer::initBaro(int altitudeAverageCnt,bool DEBUG_OUTPUT)
{
    if(!bmp.begin(0x76)) 
    {
        if(DEBUG_OUTPUT)
            Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        return false;
    }
    if(DEBUG_OUTPUT)
        Serial.println("Found BMP sensor!");
    aAC=altitudeAverageCnt;
    barometerCalibration();

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    return true;

}
