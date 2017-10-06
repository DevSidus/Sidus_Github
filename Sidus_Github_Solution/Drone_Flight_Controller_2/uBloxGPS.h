/*
uBloxGPS.h
*/

#ifndef UBLOXGPS_h
#define UBLOXGPS_h

#include "Arduino.h"				

struct gpsData {
  unsigned long   iTOW;			  ///< [ms], GPS time of the navigation epoch
  unsigned short  utcYear;		  ///< [year], Year (UTC)
  unsigned char   utcMonth;		  ///< [month], Month, range 1..12 (UTC)
  unsigned char   utcDay;		  ///< [day], Day of month, range 1..31 (UTC)
  unsigned char   utcHour;		  ///< [hour], Hour of day, range 0..23 (UTC)
  unsigned char   utcMin;		  ///< [min], Minute of hour, range 0..59 (UTC)
  unsigned char   utcSec;		  ///< [s], Seconds of minute, range 0..60 (UTC)
  unsigned char   valid;		  ///< [ND], Validity flags
  unsigned long   tAcc;			  ///< [ns], Time accuracy estimate (UTC)
  long            utcNano;		  ///< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
  unsigned char   fixType;		  ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
  unsigned char   flags;		  ///< [ND], Fix status flags
  unsigned char   flags2;		  ///< [ND], Additional flags
  unsigned char   numSV;		  ///< [ND], Number of satellites used in Nav Solution
  double          lon;			  ///< [deg], Longitude
  double          lat;			  ///< [deg], Latitude
  double          height;		  ///< [m], Height above ellipsoid 
  double          hMSL;			  ///< [m], Height above mean sea level
  double          hAcc;			  ///< [m], Horizontal accuracy estimate
  double          vAcc;			  ///< [m], Vertical accuracy estimate
  double          velN;			  ///< [m/s], NED north velocity
  double          velE;			  ///< [m/s], NED east velocity
  double          velD;			  ///< [m/s], NED down velocity
  double          gSpeed;		  ///< [m/s], Ground Speed (2-D)
  double          heading;		  ///< [deg], Heading of motion (2-D)
  double          sAcc;			  ///< [m/s], Speed accuracy estimate
  double          headingAcc;	  ///< [deg], Heading accuracy estimate (both motion and vehicle)
  double          pDOP;			  ///< [ND], Position DOP
  double		  headVeh;		  ///< [deg], Heading of vehicle (2-D)
};

class uBloxGps{
  public:
    uBloxGps();
    void read(gpsData*);
    bool parse(uint8_t);
  private:
  	uint8_t _fpos;
  	static const uint8_t _payloadSize = 96;
  	uint8_t _gpsPayload[_payloadSize];
	void calcChecksum(unsigned char* CK, unsigned char* payload, uint8_t length);
};

#endif
