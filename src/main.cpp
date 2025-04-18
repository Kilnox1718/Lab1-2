#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr char WIFI_SSID[] = "iPhone";
constexpr char WIFI_PASSWORD[] = "1234567890";

constexpr char TOKEN[] = "phmtl7x9eoy4zhu2kg02";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile bool ledState = false;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t thingsboardTaskHandle = NULL;

void InitWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
}

void InitOTA() {
  ArduinoOTA.setHostname("esp32-coreiot");
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

RPC_Response setLedSwitchState(const RPC_Data &data) {
  Serial.println("RPC setLedSwitchValue called.");
  bool newState = data;
  ledState = newState;
  digitalWrite(LED_PIN, newState);
  Serial.print("ledState is now: ");
  Serial.println(ledState ? "ON" : "OFF");
  attributesChanged = true;
  return RPC_Response("setLedSwitchValue", newState);
}

const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{"setLedSwitchValue", setLedSwitchState}
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true;
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes);
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes);

void telemetryTask(void *param) {
  while (true) {
    dht20.read();
    delay(100);

    float t = dht20.getTemperature();
    float h = dht20.getHumidity();

    Serial.print("Read DHT20 -> Temp: ");
    Serial.print(t);
    Serial.print(" °C, Humi: ");
    Serial.println(h);
    
    if (!isnan(t) && !isnan(h)) {
      tb.sendTelemetryData("temperature", t);
      tb.sendTelemetryData("humidity", h);
    }

    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void thingsboardTask(void *param) {
  while (true) {
    if (!tb.connected()) {
      Serial.print("Connecting to ThingsBoard at ");
      Serial.println(THINGSBOARD_SERVER);

      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect to ThingsBoard");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }

      Serial.println("Connected to ThingsBoard!");
      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
      } else {
        Serial.println("RPC subscription successful");
      }

      if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
        Serial.println("Failed to subscribe for shared attributes");
      }

      tb.Shared_Attributes_Request(attribute_shared_request_callback);
    }

    if (attributesChanged) {
      attributesChanged = false;
      tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    }

    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);
  InitWiFi();
  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  InitOTA();

  xTaskCreatePinnedToCore(telemetryTask, "Telemetry Task", 4096, NULL, 1, &telemetryTaskHandle, 1);
  xTaskCreatePinnedToCore(thingsboardTask, "ThingsBoard Task", 8192, NULL, 1, &thingsboardTaskHandle, 0);
}

void loop() {
  ArduinoOTA.handle();
  delay(10);
}
