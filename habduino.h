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


short errorstatus = 6;
/* Error Status Bit Level Field :
 Bit 0 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 1 = PSM Status 0 = PSM On 1 = PSM Off                   
 Bit 2 = Lock 0 = GPS Locked 1= Not Locked
 
 So error 1 means the everything is fine just the GPS is in pedestrian mode. 
 Below  500 meters the code puts the GPS in the more accurate pedestrian mode. 
 Above 2000 meters it switches to dynamic model 6 i.e flight mode
 */

void gps_get_data(void);
void gps_get_time(void);
void gps_get_position(void);
void gps_check_nav(void);
void gps_check_mode(void);
void gps_check_lock(void);
void setGPS_PowerSaveMode(void);
void setGps_MaxPerformanceMode(void);
void setGPS_DynamicMode6(void);
void sendUBX(char *MSG, char len);
short gps_verify_checksum(char* data, short len);

 
char  lock = 0, sats = 0, hour = 0, minute = 0, second = 0, tslf;
short GPSerror = 0, navmode = 0, psm_status = 0, lat_int = 51, lon_int = -4;
short altitude = 501, lat_dec = 0, lon_dec = 0;

//void setup() { resetGPS(); setupGPS();}

void gps_loop() {

  gps_get_time();
#if 0 // simplify for i2c testing
  if ( !(++tslf & 15) ) {
      // should not need to check very often.
      gps_check_nav();
      gps_check_mode();
      // TODO: check performance/powersave setting.
  }

  gps_check_lock();

  // none,guess,2d,*3d*,better guess,time only.
  if ((lock > 1) && (lock < 5))
      gps_get_position();

  if (lock == 3)
  {
    errorstatus &= ~(4);
    // set powersaving mode
    if( (sats > 5) && (errorstatus & (2)) )
    {
      setGPS_PowerSaveMode();
      errorstatus &= ~(2);
    }

  } else {
    errorstatus |= 4;
    if (!(errorstatus & 2)) // powersave and no lock
      setGps_MaxPerformanceMode();
    errorstatus |= 2;
  }
#endif
}

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

  // Calculate checksum ignoring the first two $s
  for (i = 5; i < len; i++)
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

void gps_ubx_checksum(char* data, short len, char* cka, char* ckb);
short gps_verify_checksum(char* data, short len)
{
  char a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return 0;
  else
    return 1;
}

void gps_ubx_checksum(char* data, short len, char* cka, char* ckb)
{
  short i;
  *cka = 0;
  *ckb = 0;
  for (i = 0; i < len; i++)
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}

void gps_check_mode()
{
    if( (navmode != 3) && (altitude <500) )
    {
      setGPS_DynamicMode3();
      errorstatus |= (1);
    }
    if( (navmode != 6) && (altitude >2000) )
    {
      setGPS_DynamicMode6();
      errorstatus &= ~(1);
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

short gps_lon()
{
    lon = ublox_lon();
    // 4 bytes of latitude/longitude (1e-7)
    // divide by 1000 to leave degrees + 4 digits and +/-5m accuracy
    if (lon < 0) {
        lon -= 500;
        lon /= 1000;
        lon_int = (short) (lon / 10000);
        lon_dec = (short) ((long)lon_int*10000 - lon);
    } else {
        lon += 500;
        lon /= 1000;
        lon_int = (short) (lon / 10000);
        lon_dec = (short) (lon - (long)lon_int*10000);
    }
    return lon_int;
}

short gps_lat()
{
    lat = ublox_lat();
    // may be less than zero, which would be bad
    lat += 500;
    lat /= 1000;
    lat_int = (short) (lat/10000);
    lat_dec = (short) (lat - (long)lat_int*10000) ;

    return lat_int;
}

short gps_alt()
{
    long alt = ublox_alt();
    // 4 bytes of altitude above MSL (mm)
    // Ignore low byte, but data is signed

    // Scale to meters (Within accuracy of GPS)
    alt >>= 8;
    alt *= 2097;
    return (short) (alt >> 13);
}

void gps_get_time()
{
  GPSerror = 0;
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67 };

  sendUBX(request, 8);
}

