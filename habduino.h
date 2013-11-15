/*
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
 Below 1000 meters the code puts the GPS in the more accurate pedestrian mode. 
 Above 2000 meters it switches to dynamic model 6 i.e flight mode
 So as an example error code 5 = 101 means GPS not locked and in pedestrian mode. 
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
short gps_verify_checksum(uint8_t* data, uint8_t len);

uint8_t buf[60]; 
uint8_t lock = 0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int GPSerror = 0,navmode = 0,psm_status = 0,lat_int=0,lon_int=0, temperature=0;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0 ,tslf=0;

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

    if(alt > maxalt)
        maxalt = alt;    
  } else {
    errorstatus |= 4;
    if (!(errorstatus & 2)) // powersave and no lock
      setGps_MaxPerformanceMode();
    errorstatus |= 2;
  }
#endif
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  short i;
  for (i=0; i<len; i++) {
  //    hal_i2c_write(MSG[i]);
  }
}

void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94,
    0xF5 };

  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

void setupGPS() {
  // Need to start in Flight Mode for mid-air reboots
  // Can set Portable Mode after getting fix
  setGPS_DynamicMode6();

  //Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  uint8_t setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08,
    0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xA0, 0xA9 };

  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  //gps_set_sucess=getUBX_ACK(setNMEAoff);
}

void setGPS_DynamicMode6()
{
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };

    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    //gps_set_sucess=getUBX_ACK(setdm6);
}

void setGPS_DynamicMode3()
{
  uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76 };

  sendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));
  //gps_set_sucess=getUBX_ACK(setdm3);
}

void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21,
    0x91 }; // Setup for Max Power Mode

  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}

#if 0
// Flush i2c before requesting ack
// may need delay to process request
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id, may be 0x00 for NAK
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  short i;
  for (i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
}
#endif

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

void gps_check_nav(void)
{
  // CFG-NAV5
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 };

  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }

  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}

void gps_get_data()
{
  short i;
  // Clear buf[i]
  for (i = 0; i<60; i++) 
      buf[i] = 0; // clearing buffer
  // read i2c
}

void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb);
short gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return 0;
  else
    return 1;
}

void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb)
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

void gps_check_mode() {
    if( (navmode != 3) && (alt <1000) )
    {
      setGPS_DynamicMode3();
      errorstatus |= (1);
    }
    if( (navmode != 6) && (alt >2000) )
    {
      setGPS_DynamicMode6();
      errorstatus &= ~(1);
    }
}

void setGPS_PowerSaveMode() {
  // Power Save Mode 
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22,
    0x92 }; // Setup for Power Save Mode (Default Cyclic 1s)

  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

void gps_check_lock()
{
  GPSerror = 0;
  // Construct NAV-SOL request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16 };

  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

void gps_get_position()
{
  GPSerror = 0;
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
    0x0A };

  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
        (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
        (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
        (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;

    // 4 bytes of latitude/longitude (1e-7)
    // divide by 1000 to leave degrees + 4 digits and +/-5m accuracy
    if (lon < 0) {
        lon -= 500;
        lon /= 1000;
        lon_int = lon / 10000;
        lon_dec = (lon_int * 10000) - lon;
    } else {
        lon += 500;
        lon /= 1000;
        lon_int = lon / 10000;
        lon_dec = lon - (lon_int * 10000);
    }
    lat += 500;
    lat /= 1000;
    lat_int = lat/10000;
    lat_dec = lat - (lat_int * 10000);

    // 4 bytes of altitude above MSL (mm)
    // Scale to meters (Within accuracy of GPS)
    alt >>= 8;
    alt *= 262;
    alt >>= 10;
  }
}

void gps_get_time()
{
  GPSerror = 0;
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67 };

  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(buf[22] > 23 || buf[23] > 59 || buf[24] > 60)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}

