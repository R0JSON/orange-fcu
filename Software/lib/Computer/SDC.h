#include <SD.h>
#include <Arduino.h>
//#include <String>
#include "Config.h"
class SDC
{
    public:
    #ifdef bartek_setup
        bool init(bool DEBUG_OUTPUT);
    #endif
    #ifdef wiktor_setup
        bool init(bool DEBUG_OUTPUT,SPIClass &spi);
    #endif
        void writeData(String s);
        void clse();
};
