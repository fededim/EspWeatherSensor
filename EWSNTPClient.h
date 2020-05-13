#pragma once

#include "Arduino.h"

#include <Udp.h>

#define SEVENZYYEARS 2208988800UL
#define NTP_PACKET_SIZE 48
#define NTP_DEFAULT_LOCAL_PORT 1337
#define DEBUG_NTPClient

class EWSNTPClient {
  private:
    UDP*          _udp;
    bool          _udpSetup       = false;

    const char*   _poolServerName = "pool.ntp.org"; // Default time server
    IPAddress     _poolServerIP;
    int           _port           = NTP_DEFAULT_LOCAL_PORT;
    long          _timeOffset     = 0;

    unsigned long _updateInterval = 60000;  // In ms

    unsigned long _currentEpoc    = 0;      // In s
    float         _current_epoc_dec = 0;    // In s, decimal part of current epoc
    float         _current_epoc_ms = 0;    // In ms, decimal part of current epoc
    uint32_t      _lastUpdate     = 0xff000000;// In ms

    uint16_t      _ntp_timeout    = 1000;    // In ms
    uint32_t      _last_fail      = 0xffff0000; // In ms

    byte          _packetBuffer[NTP_PACKET_SIZE];

    void          sendNTPPacket();

  public:
    EWSNTPClient(UDP& udp);
    EWSNTPClient(UDP& udp, long timeOffset);
    EWSNTPClient(UDP& udp, const char* poolServerName);
    EWSNTPClient(UDP& udp, const char* poolServerName, long timeOffset);
    EWSNTPClient(UDP& udp, const char* poolServerName, long timeOffset, unsigned long updateInterval);
    EWSNTPClient(UDP& udp, IPAddress poolServerIP);
    EWSNTPClient(UDP& udp, IPAddress poolServerIP, long timeOffset);
    EWSNTPClient(UDP& udp, IPAddress poolServerIP, long timeOffset, unsigned long updateInterval);

    /**
     * Set time server name
     *
     * @param poolServerName
     */
    void setPoolServerName(const char* poolServerName);

    /**
     * clear time server and set ip
     *
     * @param ServerIP
     */
    void setPoolServerIP(IPAddress server_ip);

    /**
     * Set ntp timeout, recommand not above 1000ms
     *
     * @param t_ms
     */
    void setTimeout(uint16_t t_ms);

    /**
     * Starts the underlying UDP client with the default local port
     */
    void begin();

    /**
     * Starts the underlying UDP client with the specified local port
     */
    void begin(int port);

    /**
     * This should be called in the main loop of your application. By default an update from the NTP Server is only
     * made every 60 seconds. This can be configured in the NTPClient constructor.
     *
     * @return 1(true) on updated and success
     *         0(false) on updated and failure
     *         2 on not time to update
     *         3 on it's time to update but last failed was just happen, so it decided to wait more
     */
    int8_t update();

    /**
     * This will force the update from the NTP Server.
     *
     * @return true on success, false on failure
     */
    bool forceUpdate();

    int getDay() const;
    int getHours() const;
    int getMinutes() const;
    int getSeconds() const;

    /**
     * Changes the time offset. Useful for changing timezones dynamically
     */
    void setTimeOffset(int timeOffset);

    /**
     * Set the update interval to another frequency. E.g. useful when the
     * timeOffset should not be set in the constructor
     */
    void setUpdateInterval(unsigned long updateInterval);

    /**
     * @return time formatted like `hh:mm:ss`
     */
    String getFormattedTime() const;

    /**
     * @return time in seconds since Jan. 1, 1970
     */
    unsigned long getEpochTime() const;

    uint64_t getEpochTimeMs() const;

    /**
     * @return ms of this second, in ms
     */
    float get_millis() const;

    /**
     * Stops the underlying UDP client
     */
    void end();
};
