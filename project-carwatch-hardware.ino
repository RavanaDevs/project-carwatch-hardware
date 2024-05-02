#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define WIFI_SSID "sanju-server"
#define WIFI_PASSWORD "12345678"
#define WIFI_ATTEMPT_COUNT 10

enum SystemState {
  STATE_TRANSMITTING,
  STATE_DEBUG,
};

SystemState currntState = STATE_DEBUG;

SoftwareSerial btSerial(D4, D3);  //TX RX
WebSocketsClient webSocket;
WiFiClient wifiClient;
StaticJsonDocument<1024> doc;

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  delay(5000);

  Serial.println("Connecting");
  connectToWIFI();
  Serial.println("\nConnected");
  Serial.println(WiFi.localIP());

  webSocket.begin("172.20.10.5", 5000, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}


String input = "";
String data = "";
byte lock = 0;
bool btSendLock = true;

static uint32_t lastTime = millis();

void loop() {
  webSocket.loop();

  switch (currntState) {
    case STATE_TRANSMITTING:
      {
        if (millis() - lastTime > 5000) {
          lastTime = millis();
          btSerial.println("atz");
          readELM("atz");
        }
        break;
      }
    case STATE_DEBUG:
      {
        readELM(input);
        break;
      }
  }
}

void readELM(String key) {
  while (btSerial.available() > 0) {
    char c = btSerial.read();
    if (c == '>') {
      doc.clear();
      doc[key] = data;
      String jsonData;
      serializeJson(doc, jsonData);fdg
      webSocket.sendTXT(jsonData);
      data = "";
      lock--;
      break;
    }
    data += c;
  }
}


void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      break;
    case WStype_CONNECTED:
      break;
    case WStype_TEXT:
      String msg = String((char*)payload);

      if (msg == "bt-unlock") {
        btSendLock = false;
        return;
      } else if (msg == "debug on") {
        currntState = STATE_DEBUG;
      } else if (msg == "debug off") {
        currntState = STATE_TRANSMITTING;
      }

      if (!btSendLock && lock == 0) {
        btSendLock = true;
        lock++;

        if (currntState == STATE_DEBUG) {
          input = msg;
          btSerial.println(msg);
        }
      }
      break;
  }
}

void connectToWIFI() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
}
