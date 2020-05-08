#include "esp32/ulp.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/sens_reg.h"
#include "soc/rtc_i2c_reg.h"
#include "esp_deep_sleep.h"
#include <soc/rtc_cntl_reg.h>
#include <soc/rtc.h>
#include <rom/rtc.h>

extern "C" {
  #include <esp_clk.h>
}

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <SPIFFS.h>
#include "FS.h"
#include <PubSubClient.h>

// Our header
#include "SensorSample.h"
#include "ulp_main.h"
#include "stdint.h"

#include "bme280.h"
#include "globaldef.h"


// ULP

// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_run_ulp(uint32_t usec);
static void start_ulp_program();
esp_err_t ulp_load_binary_ex(uint32_t load_addr, const uint8_t* program_binary, size_t program_size);


// BME280
BME280 *bme;
RTC_DATA_ATTR uint8_t dig[32];  // holds BME280 calibration data 


// NTP Synchronization variables
RTC_DATA_ATTR uint64_t rtcOffset;   // it is the number of rtc ticks from boot to the last ntp synchronization
RTC_DATA_ATTR long ntpSynchedTime;  // it is the number of seconds since 1970 to the last ntp synchronization




// RTC functions

char *TimeToString(long timestamp) {
 
  struct tm *timeinfo = localtime(&timestamp);

  return asctime(timeinfo);
}


int setUnixtime(int32_t unixtime) {
  timeval epoch = {unixtime, 0};
  return settimeofday((const timeval*)&epoch, 0);
}


/*
uint64_t rtc_time_get(void)
{
    SET_PERI_REG_MASK(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE);
    uint64_t t = READ_PERI_REG(RTC_CNTL_TIME0_REG);
    t |= ((uint64_t) READ_PERI_REG(RTC_CNTL_TIME1_REG)) << 32;
    return t;
}
*/
/*
static uint32_t esp_clk_slowclk_cal_get(void)
{
    return REG_READ(RTC_SLOW_CLK_CAL_REG);
}
*/

/*
#define RTC_XTAL_CAL_RETRY 1
#define EXT_OSC_FLAG    BIT(3)
#define CONFIG_ESP32_RTC_CLK_CAL_CYCLES 1024
#define SLOW_CLK_CAL_CYCLES  CONFIG_ESP32_RTC_CLK_CAL_CYCLES
#define MIN_32K_XTAL_CAL_VAL  15000000L


enum RtcClockSource {
    SLOW_CLK_150K = RTC_SLOW_FREQ_RTC,          //!< Internal 150 kHz RC oscillator
    SLOW_CLK_32K_XTAL = RTC_SLOW_FREQ_32K_XTAL, //!< External 32 kHz XTAL
    SLOW_CLK_8MD256 = RTC_SLOW_FREQ_8MD256,     //!< Internal 8 MHz RC oscillator, divided by 256
    SLOW_CLK_32K_EXT_OSC = RTC_SLOW_FREQ_32K_XTAL | EXT_OSC_FLAG //!< External 32k oscillator connected to 32K_XP pin
} ;



static void SetRTCClockSource(int slow_clk)
{
    rtc_slow_freq_t rtc_slow_freq = (rtc_slow_freq_t) (slow_clk & RTC_CNTL_ANA_CLK_RTC_SEL_V);
    uint32_t cal_val = 0;
    // number of times to repeat 32k XTAL calibration
    // before giving up and switching to the internal RC
    //
    int retry_32k_xtal = RTC_XTAL_CAL_RETRY;

    do {
        if (rtc_slow_freq == RTC_SLOW_FREQ_32K_XTAL) {
            /* 32k XTAL oscillator needs to be enabled and running before it can
             * be used. Hardware doesn't have a direct way of checking if the
             * oscillator is running. Here we use rtc_clk_cal function to count
             * the number of main XTAL cycles in the given number of 32k XTAL
             * oscillator cycles. If the 32k XTAL has not started up, calibration
             * will time out, returning 0.
             //
            if (slow_clk == SLOW_CLK_32K_XTAL) {
                rtc_clk_32k_enable(true);
            } else if (slow_clk == SLOW_CLK_32K_EXT_OSC) {
                rtc_clk_32k_enable_external();
            }
            // When SLOW_CLK_CAL_CYCLES is set to 0, clock calibration will not be performed at startup.
            if (SLOW_CLK_CAL_CYCLES > 0) {
                cal_val = rtc_clk_cal(RTC_CAL_32K_XTAL, SLOW_CLK_CAL_CYCLES);
                if (cal_val == 0 || cal_val < MIN_32K_XTAL_CAL_VAL) {
                    if (retry_32k_xtal-- > 0) {
                        continue;
                    }
                    rtc_slow_freq = RTC_SLOW_FREQ_RTC;
                }
            }
        } else if (rtc_slow_freq == RTC_SLOW_FREQ_8MD256) {
            SERDBG("Setting 8MD256 as clock\n");
            rtc_clk_8m_enable(true, true);
        }
        rtc_clk_slow_freq_set(rtc_slow_freq);

        if (SLOW_CLK_CAL_CYCLES > 0) {
            /* TODO: 32k XTAL oscillator has some frequency drift at startup.
             * Improve calibration routine to wait until the frequency is stable.
             //
            cal_val = rtc_clk_cal(RTC_CAL_RTC_MUX, SLOW_CLK_CAL_CYCLES);
        } else {
            const uint64_t cal_dividend = (1ULL << RTC_CLK_CAL_FRACT) * 1000000ULL;
            cal_val = (uint32_t) (cal_dividend / rtc_clk_slow_freq_get_hz());
        }
    } while (cal_val == 0);
    SERDBG("RTC_SLOW_CLK calibration value: %d\n", cal_val);
    esp_clk_slowclk_cal_set(cal_val);
}
*/



static uint64_t get_rtc_time_us(uint64_t ticks)
{
    if (ticks==0)
      ticks = rtc_time_get();
    const uint32_t cal = esp_clk_slowclk_cal_get();
    /* RTC counter result is up to 2^48, calibration factor is up to 2^24,
     * for a 32kHz clock. We need to calculate (assuming no overflow):
     *   (ticks * cal) >> RTC_CLK_CAL_FRACT
     *
     * An overflow in the (ticks * cal) multiplication would cause time to
     * wrap around after approximately 13 days, which is probably not enough
     * for some applications.
     * Therefore multiplication is split into two terms, for the lower 32-bit
     * and the upper 16-bit parts of "ticks", i.e.:
     *   ((ticks_low + 2^32 * ticks_high) * cal) >> RTC_CLK_CAL_FRACT
     */
    const uint64_t ticks_low = ticks & UINT32_MAX;
    const uint64_t ticks_high = ticks >> 32;
    return ((ticks_low * cal) >> RTC_CLK_CAL_FRACT) +
           ((ticks_high * cal) << (32 - RTC_CLK_CAL_FRACT));
}


// End RTC functions


void ConnectWifi() {
  int status = WL_IDLE_STATUS;     // the Wifi radio's status

  SERDBG("Attempting to connect to WPA SSID: %s\n",SSID);
  WiFi.mode(WIFI_STA);
  status = WiFi.begin(SSID, SSID_PASSWORD);
  
  while (status != WL_CONNECTED) {
      delay(500);

      status = WiFi.status();
      SERDBG("Status=%d\n",status);
  }

}

void DisconnectWifi() {
    WiFi.disconnect();
}



void DoNTPSynch() {
    WiFiUDP wifiUdp;
    NTPClient timeClient(wifiUdp, NTP_SERVER,0, 60000);

    SERDBG("Performing NTP query...\n");
    timeClient.begin();
    while (!timeClient.update()) {
      SERDBG("Could not get time from NTP, retrying...\n");
      delay(100);
    }  

    ntpSynchedTime=timeClient.getEpochTime();
    setUnixtime(ntpSynchedTime);
    rtcOffset=rtc_time_get();

    SERDBG("Successfully set time to %s\n",TimeToString(ntpSynchedTime));
}


void SendDataToMQTTServer(File f) {
  SensorSample data;
  WiFiClient wifiClient;
  PubSubClient clientMQTT(wifiClient);
  int pubSamples=0;


  SERDBG("SendDataToMQTTServer starts...\n");
  
  ConnectWifi();

  DoNTPSynch();   // we peform NTP synch since we are online
  
  clientMQTT.setServer(MQTT_SERVER, MQTT_PORT);

  SERDBG("Attempting MQTT connection...");
  while (!clientMQTT.connect(MQTT_CLIENTID,MQTT_USER,MQTT_PASSWD)) {
      SERDBG(".(state %d)",clientMQTT.state());
      delay(500);
  }

  SERDBG("connected\n");

  f.seek(0,SeekMode::SeekSet);
  while (f.read((byte *) &data,sizeof(SensorSample))==sizeof(SensorSample)) {
    //clientMQTT.publish("outTopic", "hello world");
    //SERDBG("Published Sample[%d]: Time %s Temp %f Press %f Hum %f\n",pubSamples,TimeToString(data.timestamp),data.temperature,data.pressure,data.humidity);
    pubSamples++;
  }

  clientMQTT.disconnect();

  DisconnectWifi();

  SERDBG("SendDataToMQTTServer ends, published %d samples...\n",pubSamples);
}



void AppendToSPIFFS() {
  long now;
  RawSensorSample rawdata[MAX_BME280_SAMPLES];
  
  time(&now);
  SERDBG("AppendToSPIFFS starts %s\n",TimeToString(now));

  uint32_t *sdata=&ulp_sdata;

  // we need copy as fast as possible the data from RTC memory in order to save it from ULP next run (it would be overwritten).
  // no processing is done here
  // it takes around 1ms to copy 180 samples
  for (int i=0;i<MAX_BME280_SAMPLES;i++) {
    int offset=i*8;

    rawdata[i].rawTimestamp=(((uint64_t) (sdata[offset+0]&0xFFFF))<<32)|((sdata[offset+1]&0xFFFF)<<16)|(sdata[offset+2]&0xFFFF);
    rawdata[i].rawPressure = ((sdata[offset+3]&0xFFFF)<<4)|(sdata[offset+4]&0xF);
    rawdata[i].rawTemperature = ((sdata[offset+5]&0xFFFF)<<4)|(sdata[offset+6]&0xF);
    rawdata[i].rawHumidity = (uint16_t) (sdata[offset+7]&0xFFFF);     
  }

  SERDBG("Finished copying %d samples from RTC memory\n",MAX_BME280_SAMPLES);  

  SPIFFS.begin(false);        // fs init has already been done in the beginning of the setup function, we can't wait for format. Even without format, call is slow, between 200-300 ms are needed.
  File f=SPIFFS.open("/SensorData.bin","a+");

  bme=new BME280(dig);
  SensorSample *data=new SensorSample();

  uint32_t calTicks=esp_clk_slowclk_cal_get();
  
  // it takes around 1.6s to write 180 samples, most of the time spent is due SERDBG (can be commented out in production environment).
  // it takes around ms to write 180 samples without SERDBG
  for (int i=0;i<MAX_BME280_SAMPLES;i++) {
    //data->timestamp=(ntpSynchedTime+get_rtc_time_us(rawdata[i].rawTimestamp-rtcOffset)/1000000);
    data->timestamp=(ntpSynchedTime+rtc_time_slowclk_to_us(rawdata[i].rawTimestamp-rtcOffset,calTicks)/1000000);
    data->temperature=bme->CalculateTemperatureFloat(rawdata[i].rawTemperature);
    data->pressure=bme->CalculatePressureFloat(rawdata[i].rawPressure);
    data->humidity=bme->CalculateHumidityFloat(rawdata[i].rawHumidity);
    
    f.write((byte *) &data,sizeof(SensorSample));
    SERDBG("SPIWrite Sample %d Time %s T %f H %.3f P %.3f\n",i,TimeToString(data->timestamp),data->temperature,data->humidity,data->pressure);  
  }

  delete data;
  delete bme;
    
  int len=f.position();   // can't use .size() because it returns the size at the time the file has been opened
  SERDBG("Written to SPI %d Samples FileLen %d\n",MAX_BME280_SAMPLES,len);  

  if (len>=MAX_FILESIZE*1024) {
    //SendDataToMQTTServer(f);
    f.close();
    //SPIFFS.remove("/SensorData.bin");
  }
  else
    f.close();

  SPIFFS.end();

  time(&now);
  SERDBG("AppendToSPIFFS ends %s\n",TimeToString(now));
}


void setup() {  

    Serial.begin(SERIAL_BAUD);
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {  
        SERDBG("EspWeatherSensor starts...\n");

        SERDBG("Initing SPIFFS...\n");
        SPIFFS.begin(true); // formats spiffs if not present
        SERDBG("SPIFFS total %dKB free %dKB\n",SPIFFS.totalBytes()/1024,(SPIFFS.totalBytes()-SPIFFS.usedBytes())/1024);
        //SPIFFS.remove("/SensorData.bin");
        SPIFFS.end();

        Settings sett; // default settings
        
        bme=new BME280(dig);
        bme->WriteSettings(sett);
        bme->Initialize();
        bme->ReadCalibrationData();
        SERDBG("BME280 test read: RawTemperature %x Val %f\n",bme->ReadRawTemperature(),bme->ReadTemperatureFloat());
        SERDBG("BME280 test read: RawPressure %x Val %f\n",bme->ReadRawPressure(),bme->ReadPressureFloat());
        SERDBG("BME280 test read: RawHumidity %x Val %f\n",bme->ReadRawHumidity(),bme->ReadHumidityFloat());
        delete bme;
      
        SERDBG("BME280 CalibrationData\n--------------\n");
        for (int i=0;i<32;i++) 
          SERDBG("Dig[%d]=%x\n",i,dig[i]);       

        //SERDBG("Clock Slow Freq Get %d, setting to 8Mhz oscillator\n",rtc_clk_slow_freq_get());
        //SetRTCClockSource(RTC_SLOW_FREQ_8MD256);
        
        ConnectWifi();
        DoNTPSynch();
        DisconnectWifi();      

        // start ulp program
        init_run_ulp(ULP_SENSOR_PERIOD * 1000);
        start_ulp_program();
        ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
        esp_deep_sleep_start();
    }
    else {
        SERDBG("Deep sleep wakeup from ULP\n");
  
        uint32_t *sdata=&ulp_sdata;

        short samples=ulp_samplecount&0xFFFF;
        SERDBG("SampleCount %d\n",samples);

        // we copy to SPI
        if (samples==MAX_BME280_SAMPLES) {
          AppendToSPIFFS();
        }
  /*      else {
          bme=new BME280(dig);

          //uint64_t cpprtc=rtc_time_get();

          for (int i=0;i<samples;i++) {       
            int offset=i*8;
  
            uint64_t ulprtc=(((uint64_t) (sdata[offset+0]&0xFFFF))<<32)|((sdata[offset+1]&0xFFFF)<<16)|(sdata[offset+2]&0xFFFF);
            uint64_t ulprtcadj=ntpSynchedTime+get_rtc_time_us(ulprtc-rtcOffset)/1000000;
    
            uint32_t rawPressure = ((sdata[offset+3]&0xFFFF)<<4)|(sdata[offset+4]&0xF);
            uint32_t rawTemp = ((sdata[offset+5]&0xFFFF)<<4)|(sdata[offset+6]&0xF);
            uint32_t rawHumidity = sdata[offset+7]&0xFFFF;     
  
            float   tempFloat=bme->CalculateTemperatureFloat(rawTemp);
            float   pressFloat=bme->CalculatePressureFloat(rawPressure);
            float   humFloat=bme->CalculateHumidityFloat(rawHumidity);
  
            //float   temp=bme->CalculateTemperature(rawTemp);
            //float   press=bme->CalculatePressure(rawPressure);
            //float   hum=bme->CalculateHumidity(rawHumidity);
                
            SERDBG("Sample[%d]:  Time %s Temp %f Pressure %f Hum %f\n",i,TimeToString(ulprtcadj),tempFloat,pressFloat,humFloat);
            //SERDBG("Sample[%d]:  Temp %f TempF %f Pressure %f PressureF %f Hum %f HumF %f\n",i,temp,tempFloat,press,pressFloat,hum,humFloat);
  
            //SERDBG("ULP RawTemp %x Value %f\n", rawTemp,bme->CalculateTemperature(rawTemp));    
            //SERDBG("ULP RawPressure %x Value %f\n", rawPressure,bme->CalculatePressure(rawPressure));    
            //SERDBG("ULP RawHumidity %x Value %f\n", rawHumidity, bme->CalculateHumidity(rawHumidity));    
          }

          delete bme;
        }
        */
        SERDBG("Entering deep sleep...\n");
        ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
        esp_deep_sleep_start();
    }
}

void loop() {
    // Nothing goes here
}




// Use this function for all your first time init like setup() used to be
static void init_run_ulp(uint32_t usec) {
  
    // initialize ulp variables
    SERDBG("init_run_ulp starts...\n");
    
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, usec));

    // You MUST load the binary before setting shared variables!
//    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    esp_err_t err = ulp_load_binary_ex(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    // Setup Sofware I2C
    // TOUCH0 = GPIO4 = SCL
    // TOUCH1 = GPIO0 = SDA
    ESP_ERROR_CHECK(rtc_gpio_init(I2C_SCL));
    ESP_ERROR_CHECK(rtc_gpio_init(I2C_SDA));

    // We will be reading and writing on these gpio
    ESP_ERROR_CHECK(rtc_gpio_set_direction(I2C_SCL, RTC_GPIO_MODE_INPUT_OUTPUT));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(I2C_SDA, RTC_GPIO_MODE_INPUT_OUTPUT)); 

    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    SERDBG("init_run_ulp ends\n");
}

static void start_ulp_program() {
    SERDBG("start_ulp_program starts...\n");
    //delay(300);
    esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
    SERDBG("start_ulp_program ends...\n");
}
