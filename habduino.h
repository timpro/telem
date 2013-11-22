/*
 (c) 2013 John Greb, using GPL code from Project Habduino

 HABDuino Tracker
 http://www.habduino.org
 (c) Anthony Stirk M0UPU 
 
 August 2013 Version 1.1
 
 Credits :
 
 GPS Code from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 
 Thanks to :
 
 Phil Heron
 James Coxon
 Dave Akerman
 
 The UKHAS Community http://ukhas.org.uk
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */

void sendUBX(char *MSG, char len);

void resetGPS() {
  char set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94,
    0xF5 };

  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  char setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08,
    0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xA0, 0xA9 };

  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
}

void setGPS_DynamicMode6()
{
  char setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };

    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
}

void setGPS_DynamicMode3()
{
  char setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76 };

  sendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));
}

void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  char setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21,
    0x91 }; // Setup for Max Power Mode

  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}

uint16_t gps_CRC16_checksum (char *string, short len)
{
  uint16_t crc, i, j;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the three $s, and the *
  for (i = 3; i < len; i++)
  {
    c = string[i];
    //  crc = _crc_xmodem_update (crc, c);
        crc = crc ^ ((uint16_t)c << 8);
        for (j=0; j<8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
  }
  return crc;
}

void gps_check_mode(short altitude)
{
    if( (altitude <500) )
    {
      setGPS_DynamicMode3();
    }
    if( (altitude >2000) )
    {
      setGPS_DynamicMode6();
    }
}

void setGPS_PowerSaveMode()
{
  char setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22,
    0x92 }; // CFG-RXM Power Save Mode (Default Cyclic 1s)

  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

void ublox_pvt()
{
  // Request a NAV-PVT message from the GPS
  char request[8] = {
    0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19 };

  sendUBX(request, 8);
}



