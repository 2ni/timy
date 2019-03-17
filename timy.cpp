/*
 * some handful time functions such as position of the sun
 * or getting time from a ntp server
 *
 * TODO sunrise/sunset https://github.com/dmkishi/Dusk2Dawn/, https://github.com/millerlp/Solarlib/
 */

#include "timy.h"

TIMY::TIMY(const char* ip) {
  ntp_server = ip;
}

TIMY::TIMY() {
  ntp_server = "192.168.1.1";
}

void TIMY::set_ntp(const char* ip) {
  ntp_server = ip;
}

byte TIMY::is_leap_year(unsigned long year) {
  return ((1970+year)>0) && !((1970+year)%4) && ( ((1970+year)%100) || !((1970+year)%400) );
}

/*
 * timestamp to datetime
 * based on https://github.com/PaulStoffregen/Time/blob/master/Time.cpp
 *
 */
TIMY::Datetime TIMY::ts2dt(unsigned long timestamp) {
    static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};
    uint8_t month, monthLength;
    uint16_t year;
    unsigned long days;

    Datetime dt;
    dt.Second = timestamp % 60;
    timestamp /= 60;
    dt.Minute = timestamp % 60;
    timestamp /= 60;
    dt.Hour = timestamp % 24;
    timestamp /= 24;
    dt.Wday = ((timestamp+4)%7)+1; // sunday = 1

    year=0;
    days=0;
    while((unsigned)(days += (is_leap_year(year) ? 366 : 365)) <= timestamp) {
      year++;
    }
    dt.Year = (1970+year); // year is offset from 1970

    days -= is_leap_year(year) ? 366:365;
    timestamp -= days;

    days=0;
    month=0;
    monthLength=0;
    for (month=0; month<12; month++) {
      if (month==1) { // february
        if (is_leap_year(year)) {
          monthLength=29;
        } else {
          monthLength=28;
        }
      } else {
        monthLength = monthDays[month];
      }

      if (timestamp >= monthLength) {
        timestamp -= monthLength;
      } else {
          break;
      }
    }
    dt.Month = month + 1;  // jan is month 1
    dt.Day = timestamp + 1;     // day of month

    return dt;
}

/*
 * humanify timestamp
 */
void TIMY::ts2human(unsigned long timestamp, char *msg) {
  if (timestamp == 0) {
    msg[0] = 0;
  } else {
    Datetime dt = ts2dt(timestamp);
    sprintf(msg, "%04d-%02d-%02d %02d:%02d:%02d", dt.Year, dt.Month, dt.Day, dt.Hour, dt.Minute, dt.Second);
  }
}

/*
 * get local timestamp
 * hardcoded to CET for now
 */
unsigned long TIMY::ts2local(unsigned long timestamp) {
  TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
  TimeChangeRule CET = {"CET", Last, Sun, Oct, 3, 60};       //Central European Standard Time
  Timezone CE(CEST, CET);
  TimeChangeRule *tcr;
  return CE.toLocal(timestamp, &tcr);
}

unsigned long TIMY::get_local_timestamp() {
  return ts2local(get_timestamp());
}

/*
 * Returns unix timestamp from ntp server (utc / gmt)
 * tries 3x if no answer
 */
unsigned long TIMY::get_timestamp() {
  int trials = 0;
  unsigned long timestamp = get_timestamp_from_ntp();
  while (timestamp == 0) {
    if (++trials >= 3) return 0;
    delay(2000);
    timestamp = get_timestamp_from_ntp();
  }

  return timestamp;
}

unsigned long TIMY::get_timestamp_from_ntp() {
  WiFiUDP udp;
  udp.begin(2390);

  // Only the first four bytes of an outgoing NTP packet need to be set
  // appropriately, the rest can be whatever.
  // polling interval 6 (=2^6sec)
  // packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  // packetBuffer[1] = 0;     // Stratum, or type of clock
  // packetBuffer[2] = 6;     // Polling Interval
  // packetBuffer[3] = 0xEC;  // Peer Clock Precision
  const long ntpFirstFourBytes = 0xEC0600E3; // NTP request header

  // Clear received data from possible stray received packets
  // for some reasons the very 1st packet is not sent
  udp.beginPacket(ntp_server, 123);
  udp.write((byte *)&ntpFirstFourBytes, 48);
  udp.endPacket();
  udp.flush();

  udp.beginPacket(ntp_server, 123);
  udp.write((byte *)&ntpFirstFourBytes, 48);
  udp.endPacket();
  // Send an NTP request
  /*
  if (! (udp.beginPacket(ntp_server, 123) // 123 is the NTP port
   && udp.write((byte *)&ntpFirstFourBytes, 48) == 48
   && udp.endPacket()))
    return 0;        // sending request failed
  */

  // Wait for response; check every pollIntv ms up to maxPoll times
  const int pollIntv = 100;    // poll every this many ms
  const byte maxPoll = 10;    // poll up to this many times
  int pktLen;        // received packet length
  Serial.printf("connecting to ntp %s", ntp_server);
  for (byte i=0; i<maxPoll; i++) {
    Serial.print(".");
    pktLen = udp.parsePacket();
    if (pktLen == 48) {
      Serial.println(" connected");

      // Read and discard the first useless bytes
      // Set useless to 32 for speed; set to 40 for accuracy.
      const byte useless = 40;
      for (byte i = 0; i < useless; ++i)
	udp.read();

      // Read the integer part of sending time
      unsigned long time = udp.read();  // NTP time
      for (byte i = 1; i < 4; i++)
	time = time << 8 | udp.read();

      // Round to the nearest second if we want accuracy
      // The fractionary part is the next byte divided by 256: if it is
      // greater than 500ms we round to the next second; we also account
      // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
      // additionally, we account for how much we delayed reading the packet
      // since its arrival, which we assume on average to be pollIntv/2.
      time += (udp.read() > 115 - pollIntv/8);

      // Discard the rest of the packet
      udp.flush();

      return time - 2208988800ul;    // convert NTP time to Unix time
    }
    delay(pollIntv);
  }

  return 0; // timeout
}

/*
 * Calculate the position of the sun
 *
 * explanation from http://www.instructables.com/id/Solar-Clock-Solar-Tracker/
 * code adapted from https://gist.github.com/ndsh/01f48e70ca2314aba73840199f882cd5
 *
 * position can be verified with http://www.sunearthtools.com/dp/tools/pos_sun.php
 *
 * Elevation
 *  0  -> horizontal
 *  90 -> vertical
 *
 * Azimuth
 *  0   -> north
 *  90  -> east
 *  180 -> south
 *  270 -> west
 *
 */
TIMY::Sunpos TIMY::calc_sunpos(Datetime dt, float lat, float lon, int degree) {
  float Azimuth;
  float Elevation;

  float ZenithAngle;
  float RightAscension;
  float Declination;
  float Parallax;

  float ElapsedJulianDays;
  float DecimalHours;
  float EclipticLongitude;
  float EclipticObliquity;

  // Auxiliary variables
  float dY;
  float dX;

  // Calculate difference in days between the current Julian Day
  // and JD 2451545.0, which is noon 1 January 2000 Universal Time
  float JulianDate;
  long int liAux1;
  long int liAux2;

  // Calculate time of the day in UT decimal hours
  DecimalHours = dt.Hour + (dt.Minute / 60.0);

  // Calculate current Julian Day
  liAux1 =(dt.Month-14)/12;
  liAux2=(1461*(dt.Year + 4800 + liAux1))/4 + (367*(dt.Month - 2-12*liAux1))/12- (3*((dt.Year + 4900 + liAux1)/100))/4+dt.Day-32075;
  JulianDate=(float)(liAux2)-0.5+DecimalHours/24.0;

  // Calculate difference between current Julian Day and JD 2451545.0
  ElapsedJulianDays = JulianDate-2451545.0;

  // Calculate ecliptic coordinates (ecliptic longitude and obliquity of the
  // ecliptic in radians but without limiting the angle to be less than 2*Pi
  // (i.e., the result may be greater than 2*Pi)
  float MeanLongitude;
  float MeanAnomaly;
  float Omega;
  Omega=2.1429-0.0010394594*ElapsedJulianDays;
  MeanLongitude = 4.8950630+ 0.017202791698*ElapsedJulianDays; // Radians
  MeanAnomaly = 6.2400600+ 0.0172019699*ElapsedJulianDays;
  EclipticLongitude = MeanLongitude + 0.03341607*sin( MeanAnomaly ) + 0.00034894*sin( 2*MeanAnomaly )-0.0001134 -0.0000203*sin(Omega);
  EclipticObliquity = 0.4090928 - 6.2140e-9*ElapsedJulianDays +0.0000396*cos(Omega);

  // Calculate celestial coordinates ( right ascension and declination ) in radians
  // but without limiting the angle to be less than 2*Pi (i.e., the result may be
  // greater than 2*Pi)
  float Sin_EclipticLongitude;
  Sin_EclipticLongitude= sin( EclipticLongitude );
  dY = cos( EclipticObliquity ) * Sin_EclipticLongitude;
  dX = cos( EclipticLongitude );
  RightAscension = atan2( dY,dX );
  if( RightAscension < 0.0 ) RightAscension = RightAscension + twopi;
  Declination = asin( sin( EclipticObliquity )*Sin_EclipticLongitude );

  // Calculate local coordinates ( azimuth and zenith angle ) in degrees
  float GreenwichMeanSiderealTime;
  float LocalMeanSiderealTime;
  float LatitudeInRadians;
  float HourAngle;
  float Cos_Latitude;
  float Sin_Latitude;
  float Cos_HourAngle;
  GreenwichMeanSiderealTime = 6.6974243242 + 0.0657098283*ElapsedJulianDays + DecimalHours;
  LocalMeanSiderealTime = (GreenwichMeanSiderealTime*15 + lon)*rad;
  HourAngle = LocalMeanSiderealTime - RightAscension;
  LatitudeInRadians = lat*rad;
  Cos_Latitude = cos( LatitudeInRadians );
  Sin_Latitude = sin( LatitudeInRadians );
  Cos_HourAngle= cos( HourAngle );
  ZenithAngle = (acos( Cos_Latitude*Cos_HourAngle *cos(Declination) + sin( Declination )*Sin_Latitude));
  dY = -sin( HourAngle );
  dX = tan( Declination )*Cos_Latitude - Sin_Latitude*Cos_HourAngle;
  Azimuth = atan2( dY, dX );
  if (Azimuth < 0.0) Azimuth = Azimuth + twopi;
  if (degree) Azimuth = Azimuth/rad;

  // Parallax Correction
  Parallax=(EarthMeanRadius/AstronomicalUnit) *sin(ZenithAngle);
  /*
  ZenithAngle=(ZenithAngle + Parallax)/rad; //Zenith angle is from the top of the visible sky (thanks breaksbassbleeps)
  Elevation = (90-ZenithAngle); //Retrieve useful elevation angle from Zenith angle
  */

  ZenithAngle = (ZenithAngle + Parallax); //Zenith angle is from the top of the visible sky (thanks breaksbassbleeps)
  Elevation = (pi/2 - ZenithAngle); //Retrieve useful elevation angle from Zenith angle
  if (degree) Elevation = Elevation/rad;

  // Elevation, Azimuth are now set
  Sunpos res = { Azimuth, Elevation };
  return res;
}
