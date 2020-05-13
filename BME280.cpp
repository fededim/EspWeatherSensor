#include "BME280.h"
#include "Wire.h"
#include "globaldef.h"

// BME280 registers
#define ID_REG     0xD0
#define TEMP_DIG_ADDR 0x88
#define PRESS_DIG_ADDR 0x8E
#define HUM_DIG_ADDR1  0xA1
#define HUM_DIG_ADDR2  0xE1

#define CTRL_HUM_ADDR  0xF2
#define CTRL_MEAS_ADDR 0xF4
#define CONFIG_ADDR    0xF5

BME280::BME280(uint8_t *dig) {
  m_dig=dig;
  m_wire=new TwoWire(0);
}

BME280::~BME280() {
  delete m_wire;    // deleting wire frees also the pins for SDA & SCL
}


void BME280::Initialize() {
    m_wire->begin(I2C_SDA, I2C_SCL);
}


void BME280::ReadCalibrationData() {
  uint8_t id=-1;
  
  ReadRegister(ID_REG, &id, 1);
  printf("ReadCalibrationData: Read i2c id %x\n",id);

  if (id!=0x60) {
    printf("ReadCalibrationData: Chip is not a BME280!\n");
    return;
  }

  // Read calibration data 32 bytes
  
  // Temp. Dig
  ReadRegister(TEMP_DIG_ADDR, &m_dig[0], 6);

  // Pressure Dig
  ReadRegister(PRESS_DIG_ADDR, &m_dig[6], 18);

  // Humidity Dig 1
  ReadRegister(HUM_DIG_ADDR1, &m_dig[24], 1);
  
  // Humidity Dig 2
  ReadRegister(HUM_DIG_ADDR2, &m_dig[25],7);
}


bool BME280::WriteSettings(Settings m_settings)
{
   uint8_t ctrlHum, ctrlMeas, config;

   CalculateRegisters(m_settings,ctrlHum, ctrlMeas, config);

   WriteRegister(CTRL_HUM_ADDR, ctrlHum);
   WriteRegister(CTRL_MEAS_ADDR, ctrlMeas);
   WriteRegister(CONFIG_ADDR, config);
}


int BME280::ReadRawPressure() {
  uint8_t buf[3];
  
  ReadRegister(0xF7,buf,3);
  
  return (((uint32_t)buf[0])<<12)|(((uint16_t) buf[1])<<4)|((buf[2]>>4)&0xF);
}


int BME280::ReadRawTemperature() {
  uint8_t buf[3];
  
  ReadRegister(0xFA,buf,3);
  
  return (((uint32_t)buf[0])<<12)|(((uint16_t) buf[1])<<4)|((buf[2]>>4)&0xF);
}




int BME280::ReadRawHumidity() {
  uint8_t buf[2];
  
  ReadRegister(0xFD,buf,2);
  
  return (((uint32_t)buf[0])<<8)|buf[1];
}



float BME280::ReadHumidity() {
  return CalculateHumidity(ReadRawHumidity());
}

float BME280::ReadPressure() {
  return CalculatePressure(ReadRawPressure());
}

// must be called first, because it calculates data needed for the 2 ReadPressure and ReadHumidity
float BME280::ReadTemperature() {
  return CalculateTemperature(ReadRawTemperature());
}


float BME280::ReadHumidityFloat() {
  return CalculateHumidityFloat(ReadRawHumidity());
}

float BME280::ReadPressureFloat() {
  return CalculatePressureFloat(ReadRawPressure());
}

// must be called first, because it calculates data needed for the 2 ReadPressure and ReadHumidity
float BME280::ReadTemperatureFloat() {
  return CalculateTemperatureFloat(ReadRawTemperature());
}


void BME280::CalculateRegisters(Settings m_settings,uint8_t& ctrlHum, uint8_t& ctrlMeas, uint8_t& config)
{
   // ctrl_hum register. (ctrl_hum[2:0] = Humidity oversampling rate.)
   ctrlHum = (uint8_t)m_settings.humOSR;
   // ctrl_meas register. (ctrl_meas[7:5] = temperature oversampling rate, ctrl_meas[4:2] = pressure oversampling rate, ctrl_meas[1:0] = mode.)
   ctrlMeas = ((uint8_t)m_settings.tempOSR << 5) | ((uint8_t)m_settings.presOSR << 2) | (uint8_t)m_settings.mode;
   // config register. (config[7:5] = standby time, config[4:2] = filter, ctrl_meas[0] = spi enable.)
   config = ((uint8_t)m_settings.standbyTime << 5) | ((uint8_t)m_settings.filter << 2) | (uint8_t)m_settings.spiEnable;
}


bool BME280::WriteRegister(uint8_t addr,uint8_t data)
{
  m_wire->beginTransmission(I2C_ADDR);
  m_wire->write(addr);
  m_wire->write(data);
  m_wire->endTransmission();

  return true;
}





bool BME280::ReadRegister(uint8_t addr, uint8_t data[], uint8_t length)
{
  uint8_t ord=0;

  m_wire->beginTransmission(I2C_ADDR);
  m_wire->write(addr);
  m_wire->endTransmission();

  m_wire->requestFrom(static_cast<uint8_t>(I2C_ADDR), length);

  while(m_wire->available())
  {
    data[ord++] = m_wire->read();
  }

  return ord == length;
}



// must be executed first in order to calculate class variable t_fine
float BME280::CalculateTemperature(int32_t raw, TempUnit unit)
{
   // Code based on calibration algorthim provided by Bosch.
   int32_t var1, var2, final;
   uint16_t dig_T1 = (m_dig[1] << 8) | m_dig[0];
   int16_t   dig_T2 = (m_dig[3] << 8) | m_dig[2];
   int16_t   dig_T3 = (m_dig[5] << 8) | m_dig[4];
   var1 = ((((raw >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
   var2 = (((((raw >> 4) - ((int32_t)dig_T1)) * ((raw >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
   t_fine = var1 + var2;
   final = (t_fine * 5 + 128) >> 8;
   return unit == TempUnit_Celsius ? final/100.0 : final/100.0*9.0/5.0 + 32.0;
}
 


float BME280::CalculateHumidity(int32_t raw)
{
   // Code based on calibration algorthim provided by Bosch.
   int32_t var1;
   uint8_t   dig_H1 =   m_dig[24];
   int16_t dig_H2 = (m_dig[26] << 8) | m_dig[25];
   uint8_t   dig_H3 =   m_dig[27];
   int16_t dig_H4 = (m_dig[28] << 4) | (0x0F & m_dig[29]);
   int16_t dig_H5 = (m_dig[30] << 4) | ((m_dig[29] >> 4) & 0x0F);
   int8_t   dig_H6 =   m_dig[31];

   var1 = (t_fine - ((int32_t)76800));
   var1 = (((((raw << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) +
   ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)dig_H6)) >> 10) * (((var1 *
   ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
   ((int32_t)dig_H2) + 8192) >> 14));
   var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
   var1 = (var1 < 0 ? 0 : var1);
   var1 = (var1 > 419430400 ? 419430400 : var1);
   return ((uint32_t)(var1 >> 12))/1024.0;
}




float BME280::CalculatePressure(int32_t raw, PresUnit unit)
{
   // Code based on calibration algorthim provided by Bosch.
   int64_t var1, var2, pressure;
   float final;

   uint16_t dig_P1 = (m_dig[7]   << 8) | m_dig[6];
   int16_t   dig_P2 = (m_dig[9]   << 8) | m_dig[8];
   int16_t   dig_P3 = (m_dig[11] << 8) | m_dig[10];
   int16_t   dig_P4 = (m_dig[13] << 8) | m_dig[12];
   int16_t   dig_P5 = (m_dig[15] << 8) | m_dig[14];
   int16_t   dig_P6 = (m_dig[17] << 8) | m_dig[16];
   int16_t   dig_P7 = (m_dig[19] << 8) | m_dig[18];
   int16_t   dig_P8 = (m_dig[21] << 8) | m_dig[20];
   int16_t   dig_P9 = (m_dig[23] << 8) | m_dig[22];

   var1 = (int64_t)t_fine - 128000;
   var2 = var1 * var1 * (int64_t)dig_P6;
   var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
   var2 = var2 + (((int64_t)dig_P4) << 35);
   var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
   var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
   if (var1 == 0) { return NAN; }                                                         // Don't divide by zero.
   pressure   = 1048576 - raw;
   pressure = (((pressure << 31) - var2) * 3125)/var1;
   var1 = (((int64_t)dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
   var2 = (((int64_t)dig_P8) * pressure) >> 19;
   pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

   final = ((uint32_t)pressure)/256.0;

   // Conversion units courtesy of www.endmemo.com.
   switch(unit){
      case PresUnit_hPa: /* hPa */
         final /= 100.0;
         break;
      case PresUnit_inHg: /* inHg */
         final /= 3386.3752577878;          /* final pa * 1inHg/3386.3752577878Pa */
         break;
      case PresUnit_atm: /* atm */
         final /= 101324.99766353; /* final pa * 1 atm/101324.99766353Pa */
         break;
      case PresUnit_bar: /* bar */
         final /= 100000.0;               /* final pa * 1 bar/100kPa */
         break;
      case PresUnit_torr: /* torr */
         final /= 133.32236534674;            /* final pa * 1 torr/133.32236534674Pa */
         break;
      case PresUnit_psi: /* psi */
         final /= 6894.744825494;   /* final pa * 1psi/6894.744825494Pa */
         break;
      default: /* Pa (case: 0) */
         break;
   }
   return final;
}




// must be executed first in order to calculate class variable t_fine
float BME280::CalculateTemperatureFloat(int32_t raw, TempUnit unit) {
   float var1, var2, final;
   uint16_t dig_T1 = (m_dig[1] << 8) | m_dig[0];
   int16_t   dig_T2 = (m_dig[3] << 8) | m_dig[2];
   int16_t   dig_T3 = (m_dig[5] << 8) | m_dig[4];
   var1 =  (((float) raw)/16384.0-((float) dig_T1)/1024.0)*((float) dig_T2);
   var2 =  ((((float) raw)/131072.0-((float) dig_T1)/8192.0)*(((float) raw)/131072.0-((float) dig_T1)/8192.0))*((float) dig_T3);
   t_fine = (int32_t) var1 + var2;
   final = (var1+var2)/5120.0;

   return unit == TempUnit_Celsius ? final : final*9.0/5.0 + 32.0;
}




float BME280::CalculateHumidityFloat(int32_t raw)
{
   // Code based on calibration algorthim provided by Bosch.
   float var1;
   uint8_t   dig_H1 =   m_dig[24];
   int16_t dig_H2 = (m_dig[26] << 8) | m_dig[25];
   uint8_t   dig_H3 =   m_dig[27];
   int16_t dig_H4 = (m_dig[28] << 4) | (0x0F & m_dig[29]);
   int16_t dig_H5 = (m_dig[30] << 4) | ((m_dig[29] >> 4) & 0x0F);
   int8_t   dig_H6 =   m_dig[31];

   var1=((float) t_fine)-76800.0;
   var1=(raw-(((float) dig_H4)*64.0+((float) dig_H5)/16384.0*var1))*(((float) dig_H2)/65536.0*(1.0+((float) dig_H6)/67108864.0*var1*(1.0+((float) dig_H3)/67108864.0*var1)));
   var1=var1*(1.0-((float) dig_H1)*var1/524288.0);

   if (var1>100.0)
    var1=100.0;
   else if (var1<0.0)
    var1=0.0;   

   return var1;
}





float BME280::CalculatePressureFloat(int32_t raw, PresUnit unit)
{
   // Code based on calibration algorthim provided by Bosch.
   float var1, var2, p;
   float final;

   uint16_t dig_P1 = (m_dig[7]   << 8) | m_dig[6];
   int16_t   dig_P2 = (m_dig[9]   << 8) | m_dig[8];
   int16_t   dig_P3 = (m_dig[11] << 8) | m_dig[10];
   int16_t   dig_P4 = (m_dig[13] << 8) | m_dig[12];
   int16_t   dig_P5 = (m_dig[15] << 8) | m_dig[14];
   int16_t   dig_P6 = (m_dig[17] << 8) | m_dig[16];
   int16_t   dig_P7 = (m_dig[19] << 8) | m_dig[18];
   int16_t   dig_P8 = (m_dig[21] << 8) | m_dig[20];
   int16_t   dig_P9 = (m_dig[23] << 8) | m_dig[22];

   var1 = ((float) t_fine/2.0)-64000.0;
   var2 = var1*var1*((float) dig_P6)/32768.0;
   var2 = var2+var1*((float) dig_P5)*2.0;
   var2 =(var2/4.0)+(((float) dig_P4)*65536.0);
   var1 = (((float) dig_P3)*var1*var1/524288.0+((float) dig_P2)*var1)/524288.0;
   var1 = (1.0+var1/32768.0)*((float) dig_P1);

   if (var1==0.0)     // avoid exception caused by division by 0
    return 0;

   p = 1048576.0-(float) raw;
   p = (p-(var2/4096.0))*6250.0/var1;
   var1 = ((float) dig_P9)*p*p/2147483648.0;
   var2 = p*((float) dig_P8)/32768.0;
   final = p+(var1+var2+((float) dig_P7))/16.0;

   // Conversion units courtesy of www.endmemo.com.
   switch(unit){
      case PresUnit_hPa: /* hPa */
         final /= 100.0;
         break;
      case PresUnit_inHg: /* inHg */
         final /= 3386.3752577878;          /* final pa * 1inHg/3386.3752577878Pa */
         break;
      case PresUnit_atm: /* atm */
         final /= 101324.99766353; /* final pa * 1 atm/101324.99766353Pa */
         break;
      case PresUnit_bar: /* bar */
         final /= 100000.0;               /* final pa * 1 bar/100kPa */
         break;
      case PresUnit_torr: /* torr */
         final /= 133.32236534674;            /* final pa * 1 torr/133.32236534674Pa */
         break;
      case PresUnit_psi: /* psi */
         final /= 6894.744825494;   /* final pa * 1psi/6894.744825494Pa */
         break;
      default: /* Pa (case: 0) */
         break;
   }
   return final;
}
