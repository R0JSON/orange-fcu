#include "ErrorHandler.h"


void ErrorHandler::init(int channel)
{
    //Buzzer channel
    pinMode(channel,OUTPUT);
}


void ErrorHandler::error(int code)
{
  switch(code)
  {
    //Sensor failure (critical)
    case 0:
      while(1)
      {
        digitalWrite(5,HIGH);
        delay(100);
        digitalWrite(5,LOW);
        delay(100);
      }
      break;
    //Electrical/continuity failure
    case 1:
      while(1)
      {
        digitalWrite(5,HIGH);
        delay(200);
        digitalWrite(5,LOW);
        delay(200);
      }
      break;
    //Communication/logging failure
    case 2:
      while(1)
      {
        //digitalWrite(5,HIGH);
        delay(400);
        digitalWrite(5,LOW);
        delay(400);
      }
      break;
    //inflight data/telemetry error (non critical)
    case 3:
      while(1)
      {
        //digitalWrite(5,HIGH);
        delay(800);
        digitalWrite(5,LOW);
        delay(800);
      }
      break;
    default:
      break;
  }

}