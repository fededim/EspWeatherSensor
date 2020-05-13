#include "Wire.h"
#include "Arduino.h"


enum TempUnit
       {
          TempUnit_Celsius,
          TempUnit_Fahrenheit
       };
  
enum PresUnit
     {
        PresUnit_Pa,
        PresUnit_hPa,
        PresUnit_inHg,
        PresUnit_atm,
        PresUnit_bar,
        PresUnit_torr,
        PresUnit_psi
     };



enum OSR
     {
        OSR_Off =  0,
        OSR_X1  =  1,
        OSR_X2  =  2,
        OSR_X4  =  3,
        OSR_X8  =  4,
        OSR_X16 =  5
     };
  
enum Mode
     {
        Mode_Sleep  = 0,
        Mode_Forced = 1,
        Mode_Normal = 3
     };
  
enum StandbyTime
     {
        StandbyTime_500us   = 0,
        StandbyTime_62500us = 1,
        StandbyTime_125ms   = 2,
        StandbyTime_250ms   = 3,
        StandbyTime_50ms    = 4,
        StandbyTime_1000ms  = 5,
        StandbyTime_10ms    = 6,
        StandbyTime_20ms    = 7
     };
  
enum Filter
     {
        Filter_Off = 0,
        Filter_2   = 1,
        Filter_4   = 2,
        Filter_8   = 3,
        Filter_16  = 4
     };
  
  
enum SpiEnable
     {
        SpiEnable_False = 0,
        SpiEnable_True = 1
     };


struct Settings
     {
        Settings(
           OSR _tosr       = OSR_X1,
           OSR _hosr       = OSR_X1,
           OSR _posr       = OSR_X1,
           Mode _mode      = Mode_Forced,
           StandbyTime _st = StandbyTime_1000ms,
           Filter _filter  = Filter_Off,
           SpiEnable _se   = SpiEnable_False
        ): tempOSR(_tosr),
           humOSR(_hosr),
           presOSR(_posr),
           mode(_mode),
           standbyTime(_st),
           filter(_filter),
           spiEnable(_se) {}
  
        OSR tempOSR;
        OSR humOSR;
        OSR presOSR;
        Mode mode;
        StandbyTime standbyTime;
        Filter filter;
        SpiEnable spiEnable;
     };


class BME280
{

  public:
    
     float CalculateTemperature(int32_t raw, TempUnit unit = TempUnit_Celsius);
     float CalculateTemperatureFloat(int32_t raw, TempUnit unit = TempUnit_Celsius);
     float CalculateHumidity(int32_t raw);
     float CalculateHumidityFloat(int32_t raw);
     float CalculatePressure(int32_t raw, PresUnit unit=PresUnit_hPa);
     float CalculatePressureFloat(int32_t raw, PresUnit unit=PresUnit_hPa);
     void ReadCalibrationData();
     bool WriteSettings(Settings m_settings);
     int ReadRawPressure();
     int ReadRawTemperature();
     int ReadRawHumidity();
     float ReadHumidity();
     float ReadTemperature();   
     float ReadPressure();
     float ReadHumidityFloat();
     float ReadTemperatureFloat();   
     float ReadPressureFloat();
     void Initialize();

     BME280(uint8_t *dig);
     ~BME280();
     
    private:
      TwoWire *m_wire;
      int32_t t_fine;
      uint8_t *m_dig;  // calibration data read from BME280
      bool ReadRegister(uint8_t addr, uint8_t data[], uint8_t length);
      bool WriteRegister(uint8_t addr,uint8_t data);
      void CalculateRegisters(Settings m_settings, uint8_t& ctrlHum, uint8_t& ctrlMeas, uint8_t& config);

};
