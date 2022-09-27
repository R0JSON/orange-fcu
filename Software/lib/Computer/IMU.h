
class inertialMeasurment
{
    public:
      bool initIMU(int gCSC, bool DEBUG_OUTPUT);
      void getRawReadings(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);

    private:
      void calibrateGyro(int gyroCalibrationSampleCount);
    
    private:
      float gOX,gOY,gOZ;
};
