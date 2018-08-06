#ifdef ARDUINO_SAMD_FEATHER_M0
  #include <WiFi101.h>
#endif

#ifdef ARDUINO_ARCH_ESP8266
  #include <ESP8266WiFi.h>
#endif

#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
#endif

#ifndef ARDUINO_ARCH_WICED
  #include <PubSubClient.h>

  #define connectWiFi(args...) WiFi.begin(args)
  #define isWiFiConnected() (WiFi.status() == WL_CONNECTED)
  #define getRSSI() WiFi.RSSI()
  #define getLocalIP() WiFi.localIP()
#endif

#ifdef ARDUINO_ARCH_WICED
  #include <adafruit_feather.h>
  #include <adafruit_mqtt.h>

  #define connectWiFi(args...) Feather.connect(args)
  #define isWiFiConnected() Feather.connected()
  #define getRSSI() Feather.RSSI()
  #define getLocalIP() reverse(Feather.localIP())
#endif

#include <Adafruit_FeatherOLED_WiFi.h>
#include <RTClib.h>
#include <CircularBuffer.h>
#include <Adafruit_SHT31.h>
#include <DS2482_Temperature.h>

#ifdef ARDUINO_SAMD_FEATHER_M0
  #define HEATER_ON_PIN 11
  #define HEATER_OFF_PIN 12
  #define BUTTON_A_PIN 9
  #define BUTTON_B_PIN 6
  #define BUTTON_C_PIN 5
  #define VBAT_PIN A7
#endif

#ifdef ARDUINO_STM32_FEATHER
  #define HEATER_ON_PIN PA4
  #define HEATER_OFF_PIN PA13
  #define BUTTON_A_PIN PA15
  #define BUTTON_B_PIN PC7
  #define BUTTON_C_PIN PC5
  #define VBAT_PIN PA1
#endif

#ifdef ARDUINO_ESP8266_ESP12
  #define HEATER_ON_PIN 13
  #define HEATER_OFF_PIN 12
  #define BUTTON_A_PIN 0
  #define BUTTON_B_PIN 16
  #define BUTTON_C_PIN 2
  #define VBAT_PIN A0
#endif

#ifdef ARDUINO_FEATHER_ESP32
  #define HEATER_ON_PIN 27
  #define HEATER_OFF_PIN 12
  #define BUTTON_A_PIN 15
  #define BUTTON_B_PIN 32
  #define BUTTON_C_PIN 14
  #define VBAT_PIN A13
#endif

#include "secrets.h"

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASSWORD;

const char* broker = BROKER;
const char* clientId = CLIENT_ID;

const int timezone = -5;

typedef struct {
  DateTime timestamp;
  char topic[50];
  char message[25];
} Message;

CircularBuffer<Message, 100> messages;

#ifndef ARDUINO_ARCH_WICED
  WiFiClient wifiClient;
  PubSubClient mqtt(wifiClient);

  #define connectMQTT(id, args...) (mqtt.setServer(args), mqtt.connect(id))
  #define subscribeMQTT(topic, handler) (mqtt.setCallback(handler), mqtt.subscribe(topic))
#endif

#ifdef ARDUINO_ARCH_WICED
  AdafruitMQTT mqtt;

  #define connectMQTT(id, args...) (mqtt.clientID(id), mqtt.connect(args))
  #define subscribeMQTT(topic, handler) mqtt.subscribe(topic, MQTT_QOS_AT_MOST_ONCE, handler)
#endif

Adafruit_FeatherOLED_WiFi display = Adafruit_FeatherOLED_WiFi();
RTC_PCF8523 rtc;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
DS2482_Temperature ds2482 = DS2482_Temperature();

bool heaterOn = false;
bool displayOn = true;

void setup() {
  pinMode(HEATER_ON_PIN, OUTPUT);
  pinMode(HEATER_OFF_PIN, OUTPUT);

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_C_PIN, INPUT_PULLUP);

  #ifdef ARDUINO_STM32_FEATHER
  pinMode(VBAT_PIN, INPUT_ANALOG);
  #endif

  display.init();
  display.setRSSIIcon(true);
  display.setRSSIAsPercentage(true);
  display.clearDisplay();

  if (!rtc.begin()) {
    display.println("RTC not present");
    display.display();
    while (true);
  }

  #ifdef ARDUINO_FEATHER_ESP32
  configTime(0, 3600, "pool.ntp.org");
  #endif

  if (!sht31.begin(0x44)) {
    display.println("SHT31-D not present");
    display.display();
    while (true);
  }

  ds2482.init();

  #ifdef ARDUINO_SAMD_FEATHER_M0
  WiFi.setPins(8, 7, 4, 2);

  if (WiFi.status() == WL_NO_SHIELD) {
    display.println("WiFi not present");
    display.display();
    while (true);
  }
  #endif

  #ifdef ARDUINO_ARCH_ESP8266
  WiFi.mode(WIFI_STA);
  #endif

  display.clearMsgArea();
  display.print("Connecting to WiFi...");
  display.display();

  connectWiFi(ssid, password);

  do {
    delay(1000);
  } while (!isWiFiConnected());

  display.clearMsgArea();
  display.print("Connecting to MQTT...");
  display.display();

  connectMQTT(clientId, broker, 1883);

  do {
    delay(1000);
  } while (!mqtt.connected());

  display.clearMsgArea();

  setHeaterState();

  subscribeMQTT("control/heating/state", handler);
}

#ifndef ARDUINO_ARCH_WICED
void handler(char* topic, byte* payload, unsigned int length) {
  if (strncmp((char*)payload, "on", length) == 0) {
    setHeaterState(true);
  }

  if (strncmp((char*)payload, "off", length) == 0) {
    setHeaterState(false);
  }
}
#endif

#ifdef ARDUINO_ARCH_WICED
void handler(UTF8String topic, UTF8String payload) {
  if (payload == "on") {
    setHeaterState(true);
  }

  if (payload == "off") {
    setHeaterState(false);
  }
}
#endif

void loop() {
  checkConnection();
  checkClock();

  readSensors();

  #ifndef ARDUINO_ARCH_WICED
  mqtt.loop();
  #endif

  updateUserInterface();
}

void checkConnection() {
  unsigned long now = millis();
  static unsigned long last = 0;

  if (now - last >= 10000) {
    if (!isWiFiConnected()) {
      connectWiFi(ssid, password);
    }

    if (isWiFiConnected() && !mqtt.connected()) {
      connectMQTT(clientId, broker, 1883);
    }

    last = now;
  }
}

DateTime getLocalTime() {
  return rtc.now() + TimeSpan(0, timezone, 0, 0);
}

DateTime getInternetTime() {
  time_t now;

  #ifdef ARDUINO_SAMD_FEATHER_M0
  now = WiFi.getTime();
  #endif

  #ifdef ARDUINO_STM32_FEATHER
  now = Feather.getUtcTime();
  #endif

  #ifdef ARDUINO_ESP8266_ESP12
  // TODO
  #endif

  #ifdef ARDUINO_FEATHER_ESP32
  time(&now);
  #endif

  return DateTime(now);
}

void checkClock() {
  unsigned long now = millis();
  static unsigned long last = 0;

  if (now - last >= 3600000 && isWiFiConnected()) {
    DateTime actual = getInternetTime();
    TimeSpan drift = actual - getLocalTime();

    if (abs(drift.seconds()) > 60) {
      rtc.adjust(actual);
    }

    last = now;
  }
}

void setHeaterState() {
  byte month = getLocalTime().month();

  setHeaterState(month <= 3 || month >= 11);
}

void setHeaterState(bool state) {
  heaterOn = state;

  if (heaterOn) {
    digitalWrite(HEATER_ON_PIN, HIGH);
    delay(15);
    digitalWrite(HEATER_ON_PIN, LOW);

    publishMessage(getLocalTime(), "data/heating/state", "on");
  } else {
    digitalWrite(HEATER_OFF_PIN, HIGH);
    delay(15);
    digitalWrite(HEATER_OFF_PIN, LOW);

    publishMessage(getLocalTime(), "data/heating/state", "off");
  }
}

void readSensors() {
  unsigned long now = millis();
  static unsigned long last = 0;
  static bool done = true;

  while (mqtt.connected() && !messages.isEmpty()) {
    Message msg = messages.shift();

    publishMessage(msg.timestamp, msg.topic, msg.message);
  }

  if (now - last >= 60000) {
    readBatteryVoltage();
    readAmbientTemperature();
    readAmbientHumidity();
    done = false;

    last = now;
  }

  if (!done) {
    done = readNextOneWire();
  }
}

void readBatteryVoltage() {
  float value;

  #ifdef ARDUINO_SAMD_FEATHER_M0
  static float last = 0.0;

  if (digitalRead(BUTTON_A_PIN)) {
    last = analogRead(VBAT_PIN) * 2 * 3.3 / 1024;
    pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  }

  value = last;
  #endif

  #ifdef ARDUINO_STM32_FEATHER
  value = analogRead(VBAT_PIN) * 2 * 3.3 / 4096;
  #endif

  #ifdef ARDUINO_ESP8266_ESP12
  value = analogRead(VBAT_PIN) * 2 * 3.3 / 1024;
  #endif

  #ifdef ARDUINO_FEATHER_ESP32
  value = analogRead(VBAT_PIN) * 2 * 3.3 / 4096;
  #endif

  publishMessage(getLocalTime(), "data/heating_logger/battery_voltage", value, "V");

  display.setBattery(value);
}

void readAmbientTemperature() {
  float value = (sht31.readTemperature() * 1.8) + 32;

  publishMessage(getLocalTime(), "data/heating_logger/ambient_temperature", value, "F");
}

void readAmbientHumidity() {
  float value = sht31.readHumidity();

  publishMessage(getLocalTime(), "data/heating_logger/ambient_humidity", value, "%");
}

bool readNextOneWire() {
  static bool done = true;
  byte rom[8];

  if (done) {
    ds2482.reset();
  }

  done = ds2482.searchNext(rom);

  if (!ds2482.getErrorFlags()) {
    float value = (ds2482.readTemperature(rom) * 1.8) + 32;

    if (!ds2482.getErrorFlags()) {
      char topic[] = "data/xxxxxxxxxxxxxxxx/temperature";
      char hex[3];

      for (byte i = 0; i < 8; i++) {
        sprintf(hex, "%02X", rom[i]);
        memcpy(topic + 5 + (i * 2), hex, 2);
      }

      publishMessage(getLocalTime(), topic, value, "F");
    }
  }

  return done;
}

void publishMessage(DateTime ts, char * topic, float value, char * units) {
  char message[15];

  sprintf(message, "%7.3f %3s", value, units);

  publishMessage(ts, topic, message);
}

void publishMessage(DateTime ts, char * topic, char * message) {
  if (mqtt.connected()) {
    char payload[50];

    sprintf(payload, "%d-%02d-%02dT%02d:%02d:%02d%+03d %s", ts.year(), ts.month(), ts.day(), ts.hour(), ts.minute(), ts.second(), timezone, message);

    mqtt.publish(topic, payload);
  } else {
    Message msg;

    msg.timestamp = ts;
    strcpy(msg.topic, topic);
    strcpy(msg.message, message);

    messages.push(msg);
  }
}

void updateUserInterface() {
  unsigned long now = millis();
  static unsigned long start = now;
  static unsigned long lastRead = 0;
  static unsigned long lastUpdate = 0;

  // Turn off display after 30 seconds
  if (displayOn && now - start >= 30000) {
    displayOn = false;
  }

  // Read the buttons 10 times per second
  if (now - lastRead >= 100) {
    if (readButtons()) {
      start = now;

      if (!displayOn) {
        displayOn = true;
      }

      refreshMenu();
      display.display();
    }

    lastRead = now;
  }

  // When the display is on refresh it every second
  if (displayOn && now - lastUpdate >= 1000) {
    updateDisplay();

    lastUpdate = now;
  }

  // When the display is off clear it once
  if (!displayOn && lastUpdate != 0) {
    display.clearDisplay();
    display.display();

    lastUpdate = 0;
  }
}

bool readButtons() {
  bool buttonA = !digitalRead(BUTTON_A_PIN);
  static bool lastA = false;

  bool buttonB = !digitalRead(BUTTON_B_PIN);
  static bool lastB = false;

  bool buttonC = !digitalRead(BUTTON_C_PIN);
  static bool lastC = false;

  // On button
  if (buttonA && !lastA) {
    setHeaterState(true);
  }

  // Off button
  if (buttonC && !lastC) {
    setHeaterState(false);
  }

  lastA = buttonA;
  lastB = buttonB;
  lastC = buttonC;

  return buttonA || buttonB || buttonC;
}

void updateDisplay() {
  refreshMenu();

  if (isWiFiConnected()) {
    display.setConnected(true);
    display.setRSSI(getRSSI());
    display.setIPAddress(getLocalIP());
  } else {
    display.setConnected(false);
    display.setRSSI(0);
    display.setIPAddress(0);
  }

  display.refreshIcons();
}

void refreshMenu() {
  display.fillRect(0, 8, 128, 16, BLACK);
  display.setCursor(0, 8);

  if (heaterOn) {
    display.print("Heater On");
  } else {
    display.print("Heater Off");
  }
}

#ifdef ARDUINO_ARCH_WICED
uint32_t reverse(uint32_t x) {
  return ((x & 0xFF) << 24) |
    ((x & 0xFF00) << 8) |
    ((x & 0xFF0000) >> 8) |
    ((x & 0xFF000000) >> 24);
}
#endif
