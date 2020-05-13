// size is 16 bytes
class SensorSample  {

    public:
        uint64_t timestamp;
        float temperature;
        float humidity;
        float pressure;
};

class RawSensorSample  {

    public:
        uint64_t rawTimestamp;
        uint32_t rawTemperature;
        uint32_t rawPressure;
        uint16_t rawHumidity;
};
