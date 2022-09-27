#pragma once
#include "Pyro.h"
#include "Config.h"
void Pyro::initPyro()
{
    
  //Attach pyro channels to given pins
  pinMode(pyro_channel_pin_1,OUTPUT);
  pinMode(pyro_channel_pin_2, OUTPUT);

}



void Pyro::triggerPyro(int channel, bool DEBUG_OUTPUT)
{
  switch(channel)
  {
    case 1:
        digitalWrite(pyro_channel_pin_1,HIGH);
        if(DEBUG_OUTPUT)
            Serial.printf("Pyro %d fired!", &channel);
        delay(100);
        digitalWrite(pyro_channel_pin_1,LOW);
      break;
    case 2:
        digitalWrite(pyro_channel_pin_2,HIGH);
        if(DEBUG_OUTPUT)
            Serial.printf("Pyro %d fired!", &channel);
        delay(100);
        digitalWrite(pyro_channel_pin_2,LOW);
      break;
  }
}
