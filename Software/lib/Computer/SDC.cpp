#include "SDC.h"
#include <SD.h>
#include "Config.h"

#ifdef bartek_setup
    File data;

    bool SDC::init(bool DEBUG_OUTPUT)
    {
        if(!SD.begin(9))
        {
            if(DEBUG_OUTPUT)
                Serial.println("Could not find SD card, check if present!");
            return false;
        }

        data = SD.open("LOG.txt", FILE_WRITE);
        data.println("Timestamp,State,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,Altitude,MaxAltitude");

        if(DEBUG_OUTPUT)
            Serial.println("Found SD card!");
        return true;


    }

    void SDC::writeData(String s)
    {
        data.println(s);
    }

    void SDC::clse()
    {
        data.close();
    }
#endif
#ifdef wiktor_setup
    File data;

    bool SDC::init(bool DEBUG_OUTPUT, SPIClass &spi)
    {
        if(!SD.begin(CS, spi, 80000000))
        {
            if(DEBUG_OUTPUT && !digitalRead(SD_SWITCHPIN))
                Serial.println("Could not find SD card, check if present!");
            return false;
        }

        data = SD.open("LOG.txt", FILE_WRITE);
        data.println("Timestamp,State,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,Altitude,MaxAltitude");

        if(DEBUG_OUTPUT)
            Serial.println("Found SD card!");
        return true;
    }

    void SDC::writeData(String s)
    {
        data.println(s);
    }

    void SDC::clse()
    {
        data.close();
    }
#endif
