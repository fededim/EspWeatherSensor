#define MAX_BME280_SAMPLES  180   // Number of samples to store in RTC memory (tested thoroughly with 180, max is around 190-194 due to limited 8k memory of RTC). (RTC memory is not turned off during deep sleeps)

#define I2C_SCL GPIO_NUM_4   // GPIO04     // pin used for I2C, if you change here you need also to update macros read_SCL, write_SCL in ulp.s
#define I2C_SDA GPIO_NUM_0   // GPIO00      // pin used for I2C, if you change here you need also to update macros read_SDA, write_SDA in ulp.s
#define I2C_ADDR  0x76            // I2C Addr of BME280

#define ULP_SENSOR_PERIOD 1000    // BME280 sensor data polling period in ms, can't be less than 300. 

#define SERIAL_BAUD 115200        // Serial baud rate

#define NTP_SERVER  "europe.pool.ntp.org"  // ntp server address

#define SSID  "SSID"        // Wifi SSID to connect to
#define SSID_PASSWORD "SSID password"   // Wifi SSID password

#define MAX_FILESIZE  20          // maximum size in KB of samples stored in SPIFFS (depends on the amount of flash memory), after this size has reached all the samples are forwarded to MQTT server

#define MQTT_SERVER ""            // MQTT Server
#define MQTT_PORT   3389          // MQTT port
#define MQTT_USER   ""            // MQTT server login
#define MQTT_PASSWD ""            // MQTT server password
#define MQTT_CLIENTID "EspWeatherSensor Poggiolo"  // MQTT client Id

#define DEBUG      // makes the program output log to serial, useful for debugging, can be commented in production environment

#ifdef DEBUG
#define SERDBG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__);
#else
#define SERDBG(fmt,...)
#endif
