/**
 * The MIT License (MIT)
 * Copyright (c) 2015 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "EWSNTPClient.h"

EWSNTPClient::EWSNTPClient(UDP& udp) {
  this->_udp            = &udp;
}

EWSNTPClient::EWSNTPClient(UDP& udp, long timeOffset) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
}

EWSNTPClient::EWSNTPClient(UDP& udp, const char* poolServerName) {
  this->_udp            = &udp;
  this->_poolServerName = poolServerName;
}

EWSNTPClient::EWSNTPClient(UDP& udp, IPAddress poolServerIP) {
  this->_udp            = &udp;
  this->_poolServerIP   = poolServerIP;
  this->_poolServerName = NULL;
}

EWSNTPClient::EWSNTPClient(UDP& udp, const char* poolServerName, long timeOffset) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
}

EWSNTPClient::EWSNTPClient(UDP& udp, IPAddress poolServerIP, long timeOffset){
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerIP   = poolServerIP;
  this->_poolServerName = NULL;
}

EWSNTPClient::EWSNTPClient(UDP& udp, const char* poolServerName, long timeOffset, unsigned long updateInterval) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
  this->_updateInterval = updateInterval;
}

EWSNTPClient::EWSNTPClient(UDP& udp, IPAddress poolServerIP, long timeOffset, unsigned long updateInterval) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerIP   = poolServerIP;
  this->_poolServerName = NULL;
  this->_updateInterval = updateInterval;
}

void EWSNTPClient::begin() {
  this->begin(NTP_DEFAULT_LOCAL_PORT);
}

void EWSNTPClient::begin(int port) {
  this->_port = port;

  this->_udp->begin(this->_port);

  this->_udpSetup = true;
}

bool EWSNTPClient::forceUpdate() {
  // flush any existing packets
  while(this->_udp->parsePacket() != 0)
    this->_udp->flush();

  uint32_t tik,tok; //tik,tok to record wait time, replace timeout
  this->sendNTPPacket();
  tik=millis();
  #ifdef DEBUG_EWSNTPClient
    Serial.println("sent ntp packet");
  #endif

  // Wait till data is there or timeout...
  uint16_t cb = 0;
  do {
    delay (1); //poll more frequently to get high accuarcy
    cb = this->_udp->parsePacket();
    if ((millis()-tik)>this->_ntp_timeout){
      this->_last_fail=millis();
      return false;
    }
  } while (cb == 0);
  tok=millis();
  #ifdef DEBUG_EWSNTPClient
    Serial.println("got ntp packet.");
    Serial.print("tik, tok, (tok-tik)/2: ");
    Serial.print(tik);Serial.print(", ");
    Serial.print(tok);Serial.print(", ");
    Serial.println((tok-tik)/2.0);
  #endif

  this->_udp->read(this->_packetBuffer, NTP_PACKET_SIZE);
  uint32_t high_word,low_word;
  uint32_t receive_int,transmit_int; //integer part
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  high_word=word(this->_packetBuffer[32],this->_packetBuffer[33]);
  low_word=word(this->_packetBuffer[34],this->_packetBuffer[35]);
  receive_int=(high_word<<16) | low_word;
  high_word=word(this->_packetBuffer[40],this->_packetBuffer[41]);
  low_word=word(this->_packetBuffer[42],this->_packetBuffer[43]);
  transmit_int=(high_word<<16) | low_word;
  #ifdef DEBUG_EWSNTPClient
    Serial.print("receive_int, transmit_int: ");
    Serial.print(receive_int);Serial.print(", ");
    Serial.println(transmit_int);
  #endif

  float receive_dec=0,transmit_dec=0; //decimal part
  high_word=word(this->_packetBuffer[36],this->_packetBuffer[37]);
  receive_dec=high_word/65536.0;
  #ifdef DEBUG_EWSNTPClient
    Serial.print("receive_dec, transmit_dec: ");
    Serial.print(high_word,HEX);Serial.print(", ");
    Serial.print(receive_dec,6);Serial.print(", ");
  #endif
  high_word=word(this->_packetBuffer[44],this->_packetBuffer[45]);
  transmit_dec=high_word/65536.0;
  #ifdef DEBUG_EWSNTPClient
    Serial.print(high_word,HEX);Serial.print(", ");
    Serial.println(transmit_dec,6);
  #endif
  
  float ping_delay;
  ping_delay=(tok-tik)/1000.0-(transmit_int-receive_int)-(transmit_dec-receive_dec);
  ping_delay/=2.0;
  if(ping_delay<=0){
    Serial.println("ERROR: ping_delay < 0.0!");
  }
  
  this->_lastUpdate=tok;
  this->_currentEpoc=transmit_int - SEVENZYYEARS ;
  this->_current_epoc_dec=ping_delay+transmit_dec;
  if(this->_current_epoc_dec>1){
    this->_currentEpoc+=(int)this->_current_epoc_dec;
    this->_current_epoc_dec-=(int)this->_current_epoc_dec;
  }

  #ifdef DEBUG_EWSNTPClient
    Serial.print("current Epoc: ");
    Serial.print(this->_currentEpoc);Serial.print(" ");
    Serial.println(this->_current_epoc_dec,6);
  #endif

  return true;  // return true after successful update
}

int8_t EWSNTPClient::update() {
  uint32_t now=millis();
  if(now>=this->_lastUpdate){ //if not overflow
      if(now-this->_lastUpdate>=this->_updateInterval){
          if(now-this->_last_fail >= 1500){
              return this->forceUpdate();
          }else{
              return 3; //return 3 if last failed was just happen
          }
      }
  }else{ //if overflowed
      if(now+0xffffffff-this->_lastUpdate >= this->_updateInterval){ 
          if(now+0xffffffff-this->_last_fail >= 1500){
              return this->forceUpdate();
          }else{
              return 3;
          }
      }
  }
  return 2;   // return 2 if update does not occur
}

unsigned long EWSNTPClient::getEpochTime() const {
  return this->_timeOffset + // User offset
         this->_currentEpoc + // Epoc returned by the NTP server
         ((millis() - this->_lastUpdate + this->_current_epoc_dec*1000)/1000.0); // Time since last update
}



uint64_t EWSNTPClient::getEpochTimeMs() const {
  return  1000*((uint64_t) this->_timeOffset) + // User offset
          1000*((uint64_t) this->_currentEpoc) + // Epoc returned by the NTP server
         (millis() - this->_lastUpdate + (uint64_t) (this->_current_epoc_dec*1000)); // Time since last update
}


float EWSNTPClient::get_millis() const{
  float ms = millis() - this->_lastUpdate + this->_current_epoc_dec*1000.0;
  ms-=(int)(ms/1000)*1000;
  return ms;
}

int EWSNTPClient::getDay() const {
  return (((this->getEpochTime()  / 86400L) + 4 ) % 7); //0 is Sunday
}
int EWSNTPClient::getHours() const {
  return ((this->getEpochTime()  % 86400L) / 3600);
}
int EWSNTPClient::getMinutes() const {
  return ((this->getEpochTime() % 3600) / 60);
}
int EWSNTPClient::getSeconds() const {
  return (this->getEpochTime() % 60);
}

String EWSNTPClient::getFormattedTime() const {
  unsigned long rawTime = this->getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = rawTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return hoursStr + ":" + minuteStr + ":" + secondStr;
}

void EWSNTPClient::end() {
  this->_udp->stop();

  this->_udpSetup = false;
}

void EWSNTPClient::setTimeOffset(int timeOffset) {
  this->_timeOffset     = timeOffset;
}

void EWSNTPClient::setUpdateInterval(unsigned long updateInterval) {
  this->_updateInterval = updateInterval;
}

void EWSNTPClient::setPoolServerName(const char* poolServerName) {
  this->_poolServerName = poolServerName;
}

void EWSNTPClient::setPoolServerIP(IPAddress server_ip){
  this->_poolServerIP   = server_ip;
  this->_poolServerName = NULL;
}

void EWSNTPClient::setTimeout(uint16_t t_ms){
   this->_ntp_timeout=t_ms;
}

void EWSNTPClient::sendNTPPacket() {
  // set all bytes in the buffer to 0
  memset(this->_packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  this->_packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  this->_packetBuffer[1] = 0;     // Stratum, or type of clock
  this->_packetBuffer[2] = 6;     // Polling Interval
  this->_packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  this->_packetBuffer[12]  = 49;
  this->_packetBuffer[13]  = 0x4E;
  this->_packetBuffer[14]  = 49;
  this->_packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  if  (this->_poolServerName) {
    this->_udp->beginPacket(this->_poolServerName, 123);
  } else {
    this->_udp->beginPacket(this->_poolServerIP, 123);
  }
  this->_udp->write(this->_packetBuffer, NTP_PACKET_SIZE);
  this->_udp->endPacket();
}
