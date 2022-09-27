#include<Arduino.h>

class Pyro
{
  public:
    void initPyro();
    void triggerPyro(int channel, bool DEBUG_OUTPUT);

};  