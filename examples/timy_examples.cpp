#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <timy.h>

#define D(x) Serial.print(x)
#define DL(x) {Serial.println(x); /*client.publish(mqtt_topic, "message");*/}
#define DF(...) Serial.printf(__VA_ARGS__);

#define WIFI_TIMEOUT 10e3


TIMY timy("192.168.1.143");
ESP8266WiFiMulti wifiMulti;

void init_wifi() {
  const String chip_id = String(ESP.getChipId(), HEX);
  const char* nodename = chip_id.c_str();

  wifiMulti.addAP("<yourssid>", "<yourpw>");
  wifiMulti.addAP("<anotherssid>", "<guesswhat>");

  if (WiFi.status() == WL_CONNECTED) {
    D("Wifi already connected to: ");
    DL(WiFi.localIP());
    return;
  }

  DL();
  DF("Hello from %s\n", nodename);
  D("Connecting");

  WiFi.mode(WIFI_STA); // default is WIFI_AP_STA
  WiFi.hostname(nodename); // must be called as very 1st but with wifi on

  unsigned long startMillis = millis();
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(250);
    D(".");
    if (millis() - startMillis > WIFI_TIMEOUT) {
      DL("Timeout");
      // TODO
    }
  }

  D(" connected to ");
  Serial.print(WiFi.SSID());

  D(". IP address: ");
  DL(WiFi.localIP());

  // setup MDNS
  if (MDNS.begin(nodename, WiFi.localIP())) {
    DF("server started on http://%s.local\n\r", nodename);
  }
}

void setup() {
      // uart
    // debug 74880 or 115200
    Serial.begin(115200);
    Serial.setTimeout(2000);
    while(!Serial) { }

    // starting app
    DL("");
    DL("Booting.");

  init_wifi();
}

void loop() {
  unsigned long ts = timy.get_local_timestamp();

  char time[16] = "";
  timy.ts2human(ts, time);
  DF("current timestamp: %s\n", time);
  delay(5000);
}
