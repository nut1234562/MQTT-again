#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#define trig_pin 18
#define echo_pin 19

const char* WIFI_SSID = "AP";
const char* WIFI_PASS = "abcdefghi";

const char* MQTT_HOST = "e38d39aff5224fda97e4acd1945e69e2.s1.eu.hivemq.cloud";
const int   MQTT_PORT = 8883;
const char* MQTT_USER = "noderedfastfast";
const char* MQTT_PASS = "Abcdefghi47";
const char* MQTT_TOPIC = "esp32/demo";
constexpr float US_TO_CM = 0.0343f / 2.0f;
const uint32_t PUBLISH_PERIOD_MS = 100;

WiFiClientSecure net;
PubSubClient mqttClient(net);

float readDistanceCmOnce(uint32_t timeout_us = 30000) {
  // Ensure a clean LOW
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  // 10 µs trigger pulse
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Measure echo HIGH time; timeout keeps us from hanging
  // On ESP32, pulseIn returns 0 on timeout.
  unsigned long echo_us = pulseIn(echo_pin, HIGH, timeout_us);
  if (echo_us == 0) return NAN; // timeout / out of range
  return echo_us * US_TO_CM;
}

float median3(float a, float b, float c) {
  // Tiny branchy median; handles NaN by bubbling it out
  if (isnan(a) || isnan(b) || isnan(c)) return NAN;
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b; // median
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  while (WiFi.status() != WL_CONNECTED) delay(100);

  net.setInsecure();                 // quick start; later switch to proper CA
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
}

void connectMQTT() {
  while (!mqttClient.connected()) {
  String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("MQTT connected");
    } else {
      Serial.printf("MQTT failed, state=%d\n", mqttClient.state());
      delay(2000);
    }
  }
}

unsigned long lastPublish=0;
void loop() {

  if (WiFi.status()!=WL_CONNECTED) return;
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  uint32_t now = millis();
  if (now - lastPublish >= PUBLISH_PERIOD_MS) {
    lastPublish = now;

    float d1 = readDistanceCmOnce();
    delay(30);
    float d2 = readDistanceCmOnce();
    delay(30);
    float d3 = readDistanceCmOnce();
    float dcm = median3(d1, d2, d3);

    if (isnan(dcm)) {
      const char* payload = "{\"distance_cm\":null,\"oor\":true}";
      mqttClient.publish(MQTT_TOPIC, payload, false);  // QoS0, not retained
    } else {
      float din = dcm / 2.54f;
      char payload[96];
      snprintf(payload, sizeof(payload),
                 "{\"distance_cm\":%.2f,\"distance_in\":%.2f,\"ts\":%lu}",
                 dcm, din, (unsigned long)now);
        mqttClient.publish(MQTT_TOPIC, payload, false);
    }
    delay(200);

  /*if (millis()-lastPublish>5000) {
    lastPublish=millis();
    float v = 1.0 + ((float)esp_random()/(float)UINT32_MAX)*9.0;
    char payload[48];
    snprintf(payload, sizeof(payload), "{\"value\":%.4f}", v);
    mqttClient.publish(MQTT_TOPIC, payload);
  }*/
      }
}
