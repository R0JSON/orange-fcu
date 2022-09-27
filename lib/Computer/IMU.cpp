#include "IMU.h"
#include "Config.h"
#ifdef batrek_setup
#include <Arduino_LSM6DS3.h>
void inertialMeasurment::getRawReadings(float & ax, float & ay, float & az, float & gx, float & gy, float & gz)
{ 
    if( IMU.gyroscopeAvailable() ) 
    {
      IMU.readGyroscope(gx, gy, gz);
      gx*=PI/180.0;
      gy*=PI/180.0;
      gz*=PI/180.0;
    
      gx-=gOX;
      gy-=gOY;
      gz-=gOZ;

      
    }

    if( IMU.accelerationAvailable() ) 
    {
      IMU.readAcceleration(ax, az, ay); 

      ax*=9.81;
      ay*=9.81;
      az*=9.81;
    }
}

void inertialMeasurment::calibrateGyro(int gyroCalibrationSampleCount)
{
  float avgX,avgY,avgZ;
  float gx,gy,gz;

  //Take given number of samples 
  for(int i=0;i<gyroCalibrationSampleCount;i++)
  {
    if(IMU.gyroscopeAvailable()) 
    {
      IMU.readGyroscope(gx, gy, gz);
    }
    avgX+=gx*=PI/180.0;;
    avgY+=gy*=PI/180.0;;
    avgZ+=gz*=PI/180.0;;
    delay(10);
  }
    avgX/=(float)gyroCalibrationSampleCount;
    avgY/=(float)gyroCalibrationSampleCount;
    avgZ/=(float)gyroCalibrationSampleCount;

    gOX=avgX;
    gOY=avgY;
    gOZ=avgZ;
}

bool inertialMeasurment::initIMU(int gCSC, bool DEBUG_OUTPUT)
{
    if(!IMU.begin())
    {
        if(DEBUG_OUTPUT)
            Serial.println("Unable to initialize the LSM6DS1. Check your wiring!");
        return false;
    }
    if(DEBUG_OUTPUT)
        Serial.println("Found LSM9DS1 9DOF");

    calibrateGyro(gCSC);

    return true;
}
#endif
#ifdef wiktor_setup
  #include <MPU9250.h>
  uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;
  float aRes, gRes, mRes;
  float   SelfTest[6];
  float   gyroBias[3] = {0.96, -0.21, 0.12}, accelBias[3] = {0.00299, -0.00916, 0.00952};
  MPU9250 MPU9250(-1);
  bool inertialMeasurment::initIMU(int gCSC, bool DEBUG_OUTPUT) {
        Wire.begin();
        Wire.setClock(400000);
        delay(1000);
        MPU9250.I2Cscan();
        uint8_t c = MPU9250.getMPU9250ID();
        delay(1000);
        if (c == 0x71 || c == 0x73) {
            
            MPU9250.resetMPU9250();
            MPU9250.SelfTest(SelfTest);
            aRes = MPU9250.getAres(Ascale);
            gRes = MPU9250.getGres(Gscale);
            mRes = MPU9250.getMres(Mscale);
            MPU9250.calibrateMPU9250(gyroBias, accelBias);
            delay(1000); 
            MPU9250.initMPU9250(Ascale, Gscale, sampleRate);
            delay(2000); // add delay to see results before serial spew of data
            if(DEBUG_OUTPUT) {
              //float ax, ay, az, gx, gy, gz, mx, my, mz;
              Serial.println("Found MPU9250");
            }
          

            return true;
        } else {
            if(DEBUG_OUTPUT)
              Serial.println("Unable to initialize the MPU9255. Check your wiring!");
            return false;
            
        }
    }
  void inertialMeasurment::calibrateGyro(int gyroCalibrationSampleCount)
{
  float avgX,avgY,avgZ;
  float gx,gy,gz;

  //Take given number of samples 
  for(int i=0;i<gyroCalibrationSampleCount;i++)
  {
    int16_t MPU9250Data[7];
    MPU9250.readMPU9250Data(MPU9250Data);
    gx = (float)MPU9250Data[4]*gRes;
    gy = (float)MPU9250Data[5]*gRes;  
    gz = (float)MPU9250Data[6]*gRes;
    avgX+=gx*=PI/180.0;;
    avgY+=gy*=PI/180.0;;
    avgZ+=gz*=PI/180.0;;
    delay(10);
  }
    avgX/=(float)gyroCalibrationSampleCount;
    avgY/=(float)gyroCalibrationSampleCount;
    avgZ/=(float)gyroCalibrationSampleCount;

    gOX=avgX;
    gOY=avgY;
    gOZ=avgZ;
}

  void inertialMeasurment::getRawReadings(float & ax, float & ay, float & az, float & gx, float & gy, float & gz){
            int16_t MPU9250Data[7];
            MPU9250.readMPU9250Data(MPU9250Data);
            ax = (float)MPU9250Data[0]*aRes - accelBias[0];
            ay = (float)MPU9250Data[1]*aRes - accelBias[1];   
            az = (float)MPU9250Data[2]*aRes - accelBias[2];  
            ax*=9.81;
            ay*=9.81;
            az*=9.81;
            //Serial.printf("%.2f %.2f %.2f", ax, ay, az);
            gx = (float)MPU9250Data[4]*gRes;
            gy = (float)MPU9250Data[5]*gRes;  
            gz = (float)MPU9250Data[6]*gRes;

            gx*=PI/180.0;
            gy*=PI/180.0;
            gz*=PI/180.0;
           // Serial.printf("%.2f %.2f %.2f\n", gx, gy, gz);
            gx-=gOX;
            gy-=gOY;
            gz-=gOZ;
        }
#endif