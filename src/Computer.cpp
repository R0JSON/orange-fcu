#include <Wire.h>
#include <SPI.h>
//#include <String>
#include <SPIFlash.h>
#include "Config.h"
#include "IMU.h"
#include "Baro.h"
#include "StateMachine.h"   
#include "ErrorHandler.h"
#include "Pyro.h"
#include "ServoControl.h"
#include "SDC.h"
#include "Orientation.h"
#include "PositionEstimate.h"
//#include "LoRa.h"


/* pio libdeps 
  lib_deps = 
    https://github.com/kriswiner/MPU9250.git
*/





#define         IN_LOOP_TESTING;                    //Test your board using serial input, bypassing sensors   [FOR THE LOVE OF GODS DON'T FORGET TO TURN THIS OFF]

#ifdef  IN_LOOP_TESTING
  #include "InLoopTest.h"
#endif



/*
State machine:
-3 ---> failsafe, wait till touchdown
-2 ---> telemetry fail, will notify and abort on the ground but in flight do nothing
-1 ---> sensor fail, leads to abort
 0 ---> ground startup, waiting for sensor data
 1 ---> ground, waiting for liftoff
 2 ---> ascent, waiting for apogee
 3 ---> descent, waiting for touchdown
 4 ---> touchdown, save data and shut down

*/







//Tuning parameters, DON'T TOUCH ANYTHING ELSE!
bool            DEBUG_OUTPUT=                     true;                       //Choose if computer should work in debug mode, printing logs on serial

unsigned long   telemetryFREQ=                    2;                          //Telemetry sending frequency in HZ

float           pitchOffset=                      0;                          //Offsets of rocket and IMU orientations in deg
float           yawOffset=                        0;
float           rollOffset=                       90.0f;    

int             gyroCalibrationSampleCount=       500;                        //Amount of samples taken for gyroscope calibration. 1000 is about 5-10 seconds of calibration
float           launchDetectionTreshold=          2;                          //Launch trigger acceleration in g
int             pyroTriggerTime=                  100;                        //Pyro ON time in miliseconds
int             dataLoggingFREQ=                  2;                          //SD data logging frequency during ascent in HZ, on the ground 1/4 of that
int             buzzerFREQ=                       1;                          //frequency of buzzing during ground operation, 2 times thst after liftoff, 4 times that on apogee, 32 on error
int             altitudeAverageCnt=               10;                         //Number of measurments used in altitude averaging
float           apogeeDetectionTreshold=          2;                          //Altitude difference in meters between max reading and actual reading able to trigger apogee detection
unsigned long   mainLoopFREQ=                     30;                         //Main logic loop frequency in HZ
float           touchdownDetectionTreshold=       10;                         //Altitude at which controller assumes it landed
float           freefallAccelerationTreshold=     1;                          //Acceleration at which computer assumse it's frefallling in m/s^2
unsigned long   stabilizationDelay=               1;                          //Delay between liftoff and stabilization start in seconds
float           gyroMeasError=                    30.0f;                      //Error of gyro in degrees
float           maxAngle=                         60.0f;                      //Maximum angle before flight termination

float           kp=                               1.0f;                       //Pid tuning parameters
float           ki=                               0.9f;    
float           kd=                               0.8f;         

float           desiredPitch=                     0;                          //Angle of stabilization in degrees
float           desiredYaw=                       0;
float           desiredRoll=                      0;

//Global
unsigned long prevTMain,lastUpdate,prevTLog,prevTBuzz;
int state=0;
unsigned long timestamp;
unsigned long deltaT;

//Configuration init
int bFQ=buzzerFREQ;


//Work variables
float ax, ay, az, gx, gy, gz, pitch, yaw, roll, altitudeIMU;
float altitude, maxAltitude, prevAltitude;
float servoValueZ, servoValueX;
float velocityBaro, velocityIMUX, velocityIMUY, velocityIMUZ;

String s="";


//IN_LOOP_TESTING variables
#ifdef IN_LOOP_TESTING
  String input="";
#endif

//uint64_t thisLoopMicros = 0; // Stores microsecond timestamp for current loop
//uint64_t lastOriUpdate = 0; // Stores mircosecond timestamp for last time orientation updated

SPIFlash flash;
Barometer baro;
inertialMeasurment imu;
servoControl serwo;
//Pid pd;
ErrorHandler eHandler;
Pyro piro;
SDC sdd;
StateMachine ST;

Orientation ori; // Main orientation measurement
EulerAngles oriMeasure;
PositionEstimate Pos;

void setup() 
{
  
  
  pinMode(LED_BUILTIN, OUTPUT);
  ///initialize all the stuff

  state=1;

  if(DEBUG_OUTPUT)
  {
    Serial.begin(115200);
  }
  Serial.print("DZIALAAAA!!");
  pinMode(buzzerPin, OUTPUT);
  dab:
  digitalWrite(buzzerPin,1);
  delay(5000);
  digitalWrite(buzzerPin, 0);
  goto dab;
  eHandler.init(buzzerPin);
  
  
  
  if(!baro.initBaro(altitudeAverageCnt,DEBUG_OUTPUT))
  {
    state=-1; //SENSOR failure
  }

  if(!imu.initIMU(gyroCalibrationSampleCount,DEBUG_OUTPUT))
  {
    //state=-1; //SENSOR failure
    Serial.println("epic fail");
  }
  /*
  #ifdef wiktor_setup
    SPIClass spi = SPIClass(VSPI);
    spi.begin(SCK, MISO, MOSI, CS);
    if(!sdd.init(DEBUG_OUTPUT, spi))
    {
      state=-2; //Telemetry fail
    }
    flash.begin();
  #endif
  #ifdef bartek_setup
  if(!sdd.init(DEBUG_OUTPUT))
  {
    state=-2; //Telemetry fail
  }
  #endif
  
  */
 
  piro.initPyro();
  //pd.initPid(kp,ki,kd);
  //serwo.initServo();
  ST.initStateMachine(launchDetectionTreshold, freefallAccelerationTreshold,  apogeeDetectionTreshold,  touchdownDetectionTreshold);
  

  timestamp = lastUpdate = millis();
}


void loop() 
{
  imu.getRawReadings(ax,ay,az,gx,gy,gz);
  

  timestamp = micros(); // Get new microsecond timestamp for this loop
  
  //Timing loop stuff
  float deltaT = (float)(timestamp - lastUpdate) / 1000000.; 
  lastUpdate = timestamp;
  
  //Barometric computation

  prevAltitude = altitude;

    if(!baro.getAltitude(altitude, maxAltitude) && state!=-1)
    {
      state=-1;
    }


 // Pos.integrateVelocityAlt(velocityBaro, deltaT, altitude, prevAltitude);
  

  //AHRS
  ori.update(gx, gy, gz, deltaT);
  oriMeasure = ori.toEuler();
  
  yaw=oriMeasure.pitch*180.0f/PI;
  pitch=oriMeasure.yaw*180.0f/PI;
  roll=oriMeasure.roll*180.0f/PI;

  //Pos.integrateVelocityAcc(ax, ay, az, pitch, yaw, roll,(float) deltaT, velocityIMUX, velocityIMUY, velocityIMUZ);
  //Pos.integrateAltitudeVel(velocityIMUX, velocityIMUY, velocityIMUZ,(float) deltaT, pitch, yaw, roll, altitudeIMU);


   
  if( (micros()-prevTMain) >= (1000000./mainLoopFREQ) )
  {
   
    prevTMain=millis();
    
    //pd.PIDcompute(pitch, yaw, roll, desiredPitch, desiredYaw, desiredRoll, 1000/mainLoopFREQ, servoValueZ, servoValueX);
    //serwo.stabilize(servoValueZ, servoValueX);

  if((micros()-prevTLog) >= (1000000./dataLoggingFREQ))
  {
    prevTLog=millis();
    sdd.writeData(s);

    if(DEBUG_OUTPUT)
    {s="";
      s+= String(pitch) + ' ' +String(yaw) + ' ' +String(roll);
      flash.writeStr(flash.getAddress(sizeof(flash.sizeofStr(s))),s);
      Serial.print(pitch);Serial.print(",");Serial.print(yaw);Serial.print(",");Serial.println(roll);

    }
  }

  if( (micros()-prevTBuzz) >= (1000000./bFQ))
  {
    prevTBuzz=millis();
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(buzzerPin, LOW);
  }
  }
}
