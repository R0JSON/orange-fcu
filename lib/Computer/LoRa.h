#include<Arduino.h>

class LoRa
{
    public:
        bool initLoRa(bool DEBUG_OUTPUT);
        void sendPacket(char s);
};
