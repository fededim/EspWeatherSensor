# EspWeatherSensor (alpha)

A wifi weather sensor based on ESP32 and Bosch BME280. The software has been developed to be friendly with a battery based device since sampling is done by using the ESP32 ULP coprocessor. When all ULP limited memory is full (8KB, there is only space for 180 temperature, pressure and humidity samples) all the samples are copied over into a file in SPIFFS which in turn, after it reaches a certain size, is published to a MQTT server (usually once a day).
