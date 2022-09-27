#include<Arduino.h>

class ErrorHandler
{
  public:
    void init(int channel);
    void error(int code);
    
};