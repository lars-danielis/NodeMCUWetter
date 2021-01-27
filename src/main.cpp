// Wetterstation mit Sensoren innen und aussen
// Das Gerät für aussen pusht seine Werte nur nach ThingSpeak
// Das Gerät für innen pusht auch aber holt sich auch die Werte von aussen
// und zeigt beides auf dem e-Paper Display an

// Lars Danielis 2021
// 1.0 16.01.21 initial
// 1.1 21.01.21 Ringspeicher für 20min und 60 min Luftdruckverlaufsermittlung, Versionsausgabe beim Start
// 1.2 22.01.21 Luftdruckänderung darstellen, dazu dient folgende Tabelle:
//              Luftdruckänderung pro Stunde
//              > -1 < 1,3 gleichbleibend
//              > 1,3 < 3,3 steigend
//              > 3,3 stark steigend, Gewitter?
//              < -1 > -2 fallend
//              < -2 stark fallend, Gewitter?
//              3 Stunden Periode für 3 Stunden Symbole:
//              ___ gleichbleibend während der letzten 3 Stunden
//              //_ zwei Stunden steigend 1 Stunde gleichbleibend
//              \// ein Stunde fallend zwei Stunden steigend
//                  usw nach dem Schema
// 1.3 23.01.21 Status und Werte der Druckverlaufssignale an ThingSpeak schicken
//              Debugausgaben optimiert, Code gespart, NTP Zeit wird bis 5 mal abgerufen wenn falsch
// 1.4 25.01.21 Aussenteil erfasst 12V und schickt sie auf Feld 5 des ThingSpeak Kanals
// 1.5 26.01.21 Aussenteil rauscht stark im Temperaturwert - Filter nötig (geklont auf Surface)
//              Temperaturwerte werden alle während des 2 Min Zyklus gesammelt und der Mittelwert genutzt.
//              Zusatzlich wird der Wert über die Bibliothek Smoothed geglättet
// 1.6 26.01.21 Batteriemessung auch glätten
//              alles glätten

char versionWetterLD[21] = "Version 1.6 26.01.21";

// e-paper 2,9" 296x128
// BUSY -> D2|4, RST -> D1|5, DC -> D0|16, CS -> D8|15, CLK -> SCK|D5|14, DIN -> MOSI|D7|13, GND -> GND, 3.3V -> 3.3V

// BME280 Lufttemperatur, Luftfeuchtigkeit und Luftdruck
// SDA -> D3|0, SCL -> D4|2, GND -> GND, 3.3V -> 3.3V

//#define CONF_INNEN // Gerät für innen mit Display und BME280 Sensor
#define CONF_AUSSEN // Gerät für aussen ohne Display aber mit BME280 Sensor

#ifdef CONF_INNEN // für das e-paper Display
#include <Wire.h>
#include "icons.h"
#endif

#include <RingBuf.h>  // Für den Verlauf des Luftdrucks
#include <Smoothed.h> // glätten der Temperatur

#include <ESP8266WiFi.h> // WLAN Zugang und Sensorbibliotheken
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <ThingSpeak.h> // privater Kanal auf Thingspeak

#include <TimeLib.h>                                                         // Bibliotheken für den Zeitabgleich damit sichergestellt ist,
#include <NTPClient.h>                                                       // dass zu geraden Minuten innen und ungeraden Minuten aussen an Thinspeak übertragen wird
#include <WiFiUdp.h>                                                         // sonst kommt es zu Verlusten von Werten weil auf ThingSpeak nur alle 15 Sekunden übertragen werden darf
#define timeOffset 120                                                       // Und sich sonst die beiden Geräte in die Quere kommen können
WiFiUDP ntpUDP;                                                              // time offset to utc in minutes, 120 minutes -> CEST
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", timeOffset * 60, 60000); // Zeitserver
void syncNTP(void);

#define SECRET_CH_ID 1279135                   // replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "WVVJSGH21CQ7IOCE" // replace XYZ with your channel write API Key
#define SECRET_READ_APIKEY "F2JI7RAHEZGKI7GV"  // replace XYZ with your channel read API Key

#define DeltaT -3.5   // 3,5°C Wärmeeintrag durch das Gerät / innen
#define DeltaH -1     // 1% Abweichung Sensortoleranz
#define DeltaTa -1.25 // 1.2°C Wärmeeintrag durch das Gerät / aussen
#define DeltaHa 0     // 0% Abweichung Sensortoleranz

const char *ssid = "FRITZ!Box 6490 ld"; // WLAN Zugangsdaten
const char *password = "57731936459729772581";

unsigned long delayTime;
float h, t, p, tk, hk, tka, hka, vFil;
char temperatureCString[6]; // innen
char humidityString[6];
char pressureString[5];
char atemperatureCString[6]; // aussen
char ahumidityString[6];
char apressureString[5];

Smoothed<float> tSmoothed;
Smoothed<float> vSmoothed;
Smoothed<float> pSmoothed;
Smoothed<float> hSmoothed;

RingBuf<float, 10> pBuffer;
RingBuf<float, 1> p20Buffer;
RingBuf<float, 2> p60Buffer;
RingBuf<float, 3> p180Buffer;
float p20akt, p20pre, p20delta, p60delta;

float Tvoltage = 0.0;

Adafruit_BME280 bme; // Sensorobjekt

WiFiServer server(80);
WiFiClient client;

int BAT = A0;              //Analog channel A0 as used to measure battery voltage
float RatioFactor = 3.617; //Resistors Ration Factor

#ifdef CONF_INNEN
#define ENABLE_GxEPD2_GFX 0
#include <GxEPD2_BW.h>                 // including both doesn't hurt
#include <GxEPD2_3C.h>                 // including both doesn't hurt
#include <Fonts/FreeSerifBold12pt7b.h> // 9,12,18,24
#include <Fonts/FreeSerifBold18pt7b.h> // 9,12,18,24

#define MAX_DISPLAY_BUFFER_SIZE 800 //
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> display(GxEPD2_290(/*CS=D8*/ SS, /*DC=D0*/ 16, /*RST=D1*/ 5, /*BUSY=D2*/ 4));
#endif

unsigned long myChannelNumber = SECRET_CH_ID;
const char *myWriteAPIKey = SECRET_WRITE_APIKEY;
const char *myReadAPIKey = SECRET_READ_APIKEY;

void init_display(void);                          // Display complete refresh
void show_values(void);                           // Fill in sensor values internal and external
void LuftdruckSymbolauswahl(float, float, float); // Symbolauswahl anhand der Luftdruckverläufe der letzten drei Stunden

void setup()
{
#ifdef CONF_INNEN
  init_display(); // first display init
#endif

  Serial.begin(115200);
  delay(10);
  Wire.begin(D3, D4); //(SDA, SCL) of BME280 sensor
  Serial.println(F(""));
  Serial.println(versionWetterLD);
  Serial.print(F("BME280 test ..."));
  bool status;
  status = bme.begin(0x76);
  if (status)
  {
    Serial.println(F(" gefunden"));
  }
  else
  {
    Serial.println(F(" FEHLER - Kein BME280 gefunden!"));
  }
  // Connect to WiFi network
  Serial.print("Connecting to " + String(ssid) + " ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  ThingSpeak.begin(client);
  tSmoothed.begin(SMOOTHED_EXPONENTIAL, 20);
  vSmoothed.begin(SMOOTHED_EXPONENTIAL, 20);
  pSmoothed.begin(SMOOTHED_EXPONENTIAL, 20);
  hSmoothed.begin(SMOOTHED_EXPONENTIAL, 20);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println(F(" WiFi connected"));

  // Start the server
  server.begin();

  // sync time
  timeClient.begin();
  syncNTP();

  // Print the IP address
  Serial.print(F("Eigene IP: "));
  Serial.println(WiFi.localIP());
}

void getWeather()
{ // Sensorwerte auslesen (10s Zyklus)
  float Vvalue = 0.0, Rvalue = 0.0;

  pSmoothed.add(bme.readPressure() / 100.0);
  p = pSmoothed.get();
  dtostrf(p, 4, 0, pressureString);
  
  hSmoothed.add(bme.readHumidity());
  h = hSmoothed.get();
    
  tSmoothed.add(bme.readTemperature());
  t = tSmoothed.get();

#ifdef CONF_INNEN
  tk = t + DeltaT;
  hk = (h * (6.1078 * pow(10, ((7.5 * t) / (237.3 + t))))) / (6.1078 * pow(10, ((7.5 * (tk)) / (237.3 + tk)))) + DeltaH;
  dtostrf(tk, 5, 1, temperatureCString);
  dtostrf(hk, 5, 1, humidityString);
  tFilBuffer.push(tk);
#else
  tka = t + DeltaTa;
  hka = (h * (6.1078 * pow(10, ((7.5 * t) / (237.3 + t))))) / (6.1078 * pow(10, ((7.5 * (tka)) / (237.3 + tka)))) + DeltaHa;
  dtostrf(tka, 5, 1, temperatureCString);
  dtostrf(hka, 5, 1, humidityString);
#endif
  for (unsigned int i = 0; i < 10; i++)
  {
    Vvalue = Vvalue + analogRead(BAT); //Read analog Voltage
    delay(5);                          //ADC stable
  }
  Vvalue = (float)Vvalue / 10.0;         //Find average of 10 values
  Rvalue = (float)(Vvalue / 1024.0) * 5; //Convert Voltage in 5v factor
  Tvoltage = Rvalue * RatioFactor;       //Find original voltage by multiplying with factor
  vSmoothed.add(Tvoltage);
  vFil = vSmoothed.get();
}

unsigned long previousMillis = 0;
const long SensorInterval = 10000;           // Sensorlesezyklus
const long ClientInterval = 20000;           // Abbruchzeit falls eine Clientverbindung hängen bleibt
bool ThingSpeakConnected = false;            // 2 minütiger ThingSpeak Kontakt
bool ThingSpeakConnectedFlagDeleted = false; // 2 minütiger ThingSpeak Kontakt

void loop()
{
  unsigned long currentMillis = millis(); // Sensorwerte alle 5 Sekunden auslesen
  if (currentMillis - previousMillis >= SensorInterval)
  {
    getWeather();
    previousMillis = currentMillis;
  }

  // Zeit alle 6 Stunden aktualisieren
  if ((hour() == 3 || hour() == 9 || hour() == 15 || hour() == 21) && // if hour is 3, 9, 15 or 21 and...
      (minute() == 3 && second() == 30))
  {            // minute is 3 and second is 0....
    syncNTP(); // ...either sync using ntp or...
  }
  ESP.wdtFeed(); // feed the watchdog each time loop() is cycled through, just in case...

  // Check if a client has connected
  client = server.available();
  if (client)
  {
    boolean blank_line = true;
    while (client.connected() && not(millis() - currentMillis >= ClientInterval))
    {
      if (client.available())
      {
        char c = client.read();
        if (c == '\n' && blank_line)
        {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");
          client.println();
          // your actual web page that displays temperature
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<head><META HTTP-EQUIV=\"refresh\" CONTENT=\"15\"></head>");
#ifdef CONF_INNEN
          client.println("<body><h1>Danielis, Poststrasse 103, Eislingen Innensensoren</h1>");
#else
          client.println("<body><h1>Danielis, Poststrasse 103, Eislingen Aussensensoren</h1>");
#endif
          client.println("<table border=\"2\" width=\"600\" cellpadding=\"10\"><tbody><tr><td>");
#ifdef CONF_INNEN
          client.println("<h3>Innentemperatur Rohwert = ");
          client.println(String(t));
          client.println("&deg;C</h3><h3>Innenluftfeuchte Rohwert = ");
          client.println(String(h));
          client.println("%</h3>");
          client.println("<h3>Korrekturwert Temperatur = ");
          client.println(String(DeltaT));
          client.println("&deg;C</h3>");
          client.println("<h3>Korrekturwert Luftfeuchtigkeit = ");
          client.println(String(DeltaH));
          client.println("%</h3>");
          client.println("<h3>Innentemperatur korrigiert = ");
          client.println(temperatureCString);
          client.println("&deg;C</h3><h3>Innenluftfeuchte korrigiert = ");
          client.println(humidityString);
          client.println("%</h3>");
          client.println("<h3>Innennuftdruck = ");
          client.println(pressureString);
          client.println("hPa");
          client.println("<h3>Aussentemperatur = ");
          client.println(atemperatureCString);
          client.println("&deg;C</h3><h3>Aussenluftfeuchte = ");
          client.println(ahumidityString);
          client.println("%</h3>");
          client.println("<h3>Aussenluftdruck = ");
          client.println(apressureString);
          client.println("hPa");
          client.println("<h3>Luftdruckmittelwert des letzten 20min Intervals = ");
          client.println(p20akt);
          client.println("hPa");
          client.println("<h3>Luftdruckmittelwert des vorherigen 20min Intervals = ");
          client.println(p20pre);
          client.println("hPa");
          client.println("<h3>Luftdruckdifferenz der beiden Intervalle = ");
          client.println(p20delta);
          client.println("hPa");
          client.println("<h3>Luftdruckdifferenz der letzten Stunde = ");
          client.println(p60delta);
          client.println("hPa");
#else
          client.println("<h3>Aussentemperatur korrigiert gefiltert = ");
          client.println(temperatureCString);
          client.println("&deg;C</h3>");
          client.println("<h3>Aussenluftfeuchte korrigiert = ");
          client.println(humidityString);
          client.println("%</h3>");
          client.println("<h3>Aussentemperatur Rohwert gefiltert = ");
          client.println(String(t));
          client.println("&deg;C</h3><h3>Aussenluftfeuchte Rohwert gefiltert = ");
          client.println(String(h));
          client.println("%</h3>");
          client.println("<h3>Korrekturwert Temperatur = ");
          client.println(String(DeltaTa));
          client.println("&deg;C</h3>");
          client.println("<h3>Korrekturwert Luftfeuchtigkeit = ");
          client.println(String(DeltaHa));
          client.println("%</h3>");
          client.println("<h3>Aussenluftdruck = ");
          client.println(pressureString);
          client.println("hPa");
#endif
          client.println("</h3></td></tr></tbody></table></body></html>");
          break;
        }
        if (c == '\n')
        {
          // when starts reading a new line
          blank_line = true;
        }
        else if (c != '\r')
        {
          // when finds a character on the current line
          blank_line = false;
        }
      }
    }
    // closing the client connection
    client.stop();
  }

#ifdef CONF_INNEN
  const int modulo_innen_aussen = 0; // even
#else
  const int modulo_innen_aussen = 1; // odd
#endif

  if (not((minute() % 2) == modulo_innen_aussen) && ThingSpeakConnectedFlagDeleted)
  { // die Minute in der ThingSpeak Kontakt stattfand ist vorbei, vorbereiten für die nächste 2. Minute
    ThingSpeakConnected = false;
    ThingSpeakConnectedFlagDeleted = false; // 2 minütiger ThingSpeak Kontakt
  }

  if (((minute() % 2) == modulo_innen_aussen) && not ThingSpeakConnected && p > 0)
  {
    ThingSpeakConnected = true;
    ThingSpeakConnectedFlagDeleted = true; // 2 minütiger ThingSpeak Kontakt
    
    String status = "Druckdeltas um: " + String(hour()) + ":" + String(minute()) + ":" + String(second()) + "; "; // Statusstring Anfang

#ifdef CONF_INNEN
    float ta = ThingSpeak.readFloatField(myChannelNumber, 1, myReadAPIKey); // Sensorwerte vom Aussengerät holen
    float ha = ThingSpeak.readFloatField(myChannelNumber, 3, myReadAPIKey);
    float pa = ThingSpeak.readFloatField(myChannelNumber, 2, myReadAPIKey);
    int x = ThingSpeak.getLastReadStatus();
    if (x != 200) // Prüfen ob das Werteholen klappte
    {
      Serial.println("Problem reading channel. HTTP error code " + String(x));
    }

    if (!pBuffer.push(p))
    { // Buffer voll, dann leeren
      float pSum = 0;
      float pTemp = 0;
      Serial.println("Druckdeltas um: " + String(hour()) + ":" + String(minute()) + ":" + String(second()));
      while (pBuffer.pop(pTemp))
      { // Buffer Werte holen bis leer
        pSum = pSum + pTemp;
      }
      pBuffer.push(p); // jetzt ist Buffer leer und der aktuelle der erst nicht mehr reinpasst hat jetzt Platz
      p20akt = pSum / 10;
      Serial.println("20min: " + String(p20akt));
      if (!p20Buffer.push(p20akt))
      {                         // der erste Werte wenn der Buffer noch leer ist
        p20Buffer.pop(p20pre);  // Sonst Werte vom letzten Mal holen
        p20Buffer.push(p20akt); // und aktuellen Wert reinschreiben
        p20delta = p20akt - p20pre;
        status = status + "2 * 20min Intervalldelta: " + String(p20delta) + "; ";
        Serial.println("20minDelta: " + String(p20delta));
        if (!p60Buffer.push(p20delta))
        {
          p60delta = 0;
          while (p60Buffer.pop(pTemp))
          {
            p60delta = p60delta + pTemp;
          }
          p60delta = p60delta + p20delta;
          Serial.println("60min: " + String(p60delta));
          if (p60delta > 0)
          {
            status = status + "1h Anstieg:" + String(p60delta) + "; ";
          }
          if (p60delta < 0)
          {
            status = status + "1h Abfall:" + String(p60delta) + "; ";
          }
          if (p60delta == 0)
          {
            status = status + "t < 1h; ";
          }
          float xyz;
          if (!p180Buffer.push(p60delta))
          {
            p180Buffer.pop(xyz);
            p180Buffer.push(p60delta);
          }
          Serial.println("3h: -3: " + String(p180Buffer[0]) + ", -2: " + String(p180Buffer[1]) + ", -1:" + String(p180Buffer[2]));
          status = status + "3h: " + String(p180Buffer[0]) + ", " + String(p180Buffer[1]) + ", " + String(p180Buffer[2]) + "; ";
        }
      }
    }

    dtostrf(ta, 5, 1, atemperatureCString); // Strings für die Ausgabe bauen
    dtostrf(ha, 5, 1, ahumidityString);
    dtostrf(pa, 4, 0, apressureString);

    if (minute() == 0 || minute() == 30)
    { // alle 30 Minuten das Display refreshen
      init_display();
    }
    show_values(); // Werte anzeigen, innen und aussen
#endif

#ifdef CONF_INNEN // Werte für ThingSpeak sammeln
    ThingSpeak.setField(6, tk);
    ThingSpeak.setField(8, hk);
    ThingSpeak.setField(7, p);
    ThingSpeak.setStatus(status);
#else
    ThingSpeak.setField(1, tka);
    ThingSpeak.setField(3, hka);
    ThingSpeak.setField(2, p);
    ThingSpeak.setField(5, vFil);
    ThingSpeak.setStatus("Aussenwerte");
#endif

    int y = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (y != 200) // Werte an ThinSpeak senden
    {
      Serial.println("Problem updating channel. HTTP error code " + String(y)); // Und melden falls das schief ging
    }
  }
}

boolean Sommerzeit(void)
{
  if ((month(timeClient.getEpochTime()) < 3) || (month(timeClient.getEpochTime()) > 10))
    return 0;                                                                                                                                                          // Wenn nicht innerhalb der Sommerzeitmonate, dann gleich abbrechen
  if (((day(timeClient.getEpochTime()) - weekday(timeClient.getEpochTime())) >= 25) && (weekday(timeClient.getEpochTime()) || (hour(timeClient.getEpochTime()) >= 2))) // Wenn nach dem letzten Sonntag und nach 2:00 Uhr
  {
    if (month(timeClient.getEpochTime()) == 10)
      return 0; // dann wenn Oktober, dann gleich abbrechen
  }
  else // Wenn nicht nach dem letzten Sonntag bzw. vor 2:00 Uhr
  {
    if (month(timeClient.getEpochTime()) == 3)
      return 0; // und es ist März, dann gleich abbrechen
  }
  return 1; // ansonsten innerhalb der SZMonate, nach dem letzten Sonntag und nicht Oktober, vor dem letzten Sonntag und nicht März, dann ist SZ
}

void syncNTP()
{ // gets time from ntp and sets internal time accordingly, will return when no connection is established
  Serial.println(F("Entering syncNTP()..."));
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(F("No active WiFi connection!"));
    return;
  }            // Sometimes the connection doesn't work right away although status is WL_CONNECTED...
  delay(1500); // ...so we'll wait a moment before causing network traffic
  timeClient.update();
  int tryNTP = 5;
  while ((timeClient.getEpochTime() < 1608727253) && tryNTP)
  {
    tryNTP--;
    Serial.println(F("Falsche Zeit, ein neuer Versuch!"));
    timeClient.update();
    delay(1500);
  }
  int timediff = timeClient.getEpochTime() - now();
  if (timediff > 3000)
    timediff = timediff - 3600;
  Serial.println("Difftime ntp - nodemcu: " + String(timediff));
  Serial.println(F("syncNTP() done..."));
  if (Sommerzeit())
    setTime(timeClient.getEpochTime());
  else
    setTime(timeClient.getEpochTime() - 3600);
  Serial.println("Uhrzeit: " + String(hour()) + ":" + String(minute()) + ":" + String(second()));
}

#ifdef CONF_INNEN
void init_display() // Display refreshen und Symbole einzeichnen
{
  display.init();
  display.setFont(&FreeSerifBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.drawInvertedBitmap(7, 0, thermometer_innen, 18, 48, GxEPD_BLACK);
    display.drawInvertedBitmap(0, 54, hygrometer, 32, 40, GxEPD_BLACK);
    display.drawInvertedBitmap(0, 104, barometer, 36, 40, GxEPD_BLACK);
    display.drawLine(0, 148, 127, 148, GxEPD_BLACK);
    display.drawInvertedBitmap(7, 150, thermometer_aussen, 18, 48, GxEPD_BLACK);
    display.drawInvertedBitmap(0, 203, hygrometer, 32, 40, GxEPD_BLACK);
    //display.drawInvertedBitmap(0, 252, Luftdruck0, 39, 39, GxEPD_BLACK);
  } while (display.nextPage());
}

void show_values() // Sensorwerte für innen und aussen auf dem Display anzeigen bzw. refreshen
{
  display.setFont(&FreeSerifBold18pt7b);    // innen
  display.setPartialWindow(32, 0, 102, 48); // Temperatur
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(30, 34);
    display.print(temperatureCString);
    display.print("C");
  } while (display.nextPage());

  display.setFont(&FreeSerifBold12pt7b); // Druck und Feuchte
  display.setPartialWindow(40, 49, 87, 98);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(38, 80);
    display.print(humidityString);
    display.print("%");
    display.setCursor(38, 130);
    display.print(pressureString);
    display.print("hPa");
  } while (display.nextPage());

  display.setFont(&FreeSerifBold18pt7b);          // aussen
  display.setPartialWindow(32, 0 + 149, 102, 48); // Temperatur
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(30, 34 + 149);
    display.print(atemperatureCString);
    display.print("C");
  } while (display.nextPage());

  display.setFont(&FreeSerifBold12pt7b); // Druck und Feuchte
  display.setPartialWindow(40, 49 + 149, 87, 98);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(38, 80 + 149);
    display.print(ahumidityString);
    display.print("%");
    display.setCursor(38, 130 + 149);
    display.print(String(p60delta));
    display.print("hPa");
  } while (display.nextPage());
  LuftdruckSymbolauswahl(p180Buffer[0], p180Buffer[1], p180Buffer[2]);
}

void LuftdruckSymbolauswahl(float hr3, float hr2, float hr1)
{
  display.setPartialWindow(0, 252, 39, 39); // Temperatur
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    if ((hr3 > -1 && hr3 < 1.3) && (hr2 > -1 && hr2 < 1.3) && (hr1 > -1 && hr1 < 1.3))
    { // Gleichbleibend
      display.drawInvertedBitmap(0, 252, Luftdruck5, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 > 1.3) && (hr2 > 1.3) && (hr1 < -1))
    { // 2 * steigend und dann fallend
      display.drawInvertedBitmap(0, 252, Luftdruck1, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 > 1.3) && (hr2 > 1.3) && (hr1 > -1 && hr1 < 1.3))
    { // 2 * steigend und dann gleichbleibend
      display.drawInvertedBitmap(0, 252, Luftdruck2, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 > 1.3) && (hr2 > 1.3) && (hr1 > 1.3))
    { // 3 * steigend
      display.drawInvertedBitmap(0, 252, Luftdruck0, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 > -1) && (hr2 < -1) && (hr1 < -1))
    { // steigend oder stabil und dann 2 * fallend
      display.drawInvertedBitmap(0, 252, Luftdruck3, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 < 1.3) && (hr2 > 1.3) && (hr1 > 1.3))
    { // fallend oder stabil und dann 2 * steigend
      display.drawInvertedBitmap(0, 252, Luftdruck4, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 < -1) && (hr2 < -1) && (hr1 > -1 && hr1 < 1.3))
    { // 2 * fallend und dann gleichbleibend
      display.drawInvertedBitmap(0, 252, Luftdruck8, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 < -1) && (hr2 < -1) && (hr1 < -1))
    { // 3 * fallend
      display.drawInvertedBitmap(0, 252, Luftdruck6, 39, 39, GxEPD_BLACK);
    }
    else if ((hr3 < -1) && (hr2 < -1) && (hr1 > 1.3))
    { // 2 * fallend und dann steigend
      display.drawInvertedBitmap(0, 252, Luftdruck7, 39, 39, GxEPD_BLACK);
    }
    else
    {
      display.drawInvertedBitmap(0, 252, Luftdruck5, 39, 39, GxEPD_BLACK);
    }

  } while (display.nextPage());
}

#endif
