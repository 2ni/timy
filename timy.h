#include <math.h>
#include <stdint.h>

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#include <Timezone.h>

#define pi 3.14159265358979323846
#define twopi (2*pi)
#define rad (pi/180)
#define EarthMeanRadius 6371.01      // In km
#define AstronomicalUnit 149597890      // In km

class TIMY {

private:
  const char* ntp_server;
  unsigned long get_timestamp_from_ntp();
  byte is_leap_year(unsigned long year);


public:
  typedef struct {
    float Azimuth;
    float Elevation;
  } Sunpos;

  typedef struct  {
    uint8_t Second;
    uint8_t Minute;
    uint8_t Hour;
    uint8_t Wday;   // day of week, sunday is day 1
    uint8_t Day;
    uint8_t Month;
    uint16_t Year;   // offset from 1970;
  } Datetime;

  TIMY(const char* ntp_server);
  TIMY();
  void set_ntp(const char* ip);
  Sunpos calc_sunpos(Datetime dt, float lat, float lon, int degree=1);
  unsigned long get_timestamp();
  unsigned long get_local_timestamp();
  unsigned long ts2local(unsigned long timestamp);
  void ts2human(unsigned long timestamp, char *msg);
  Datetime ts2dt(unsigned long timestamp);
};

