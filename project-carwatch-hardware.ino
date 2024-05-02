#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define WIFI_SSID "sanju-server"
#define WIFI_PASSWORD "12345678"
#define WIFI_ATTEMPT_COUNT 10

enum SystemState {
  STATE_SETUP,
  STATE_TRANSMITTING,
  STATE_DEBUG,
};

SystemState currntState = STATE_SETUP;

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

byte readLock = 1;

byte lock = 0;
bool btSendLock = true;

static uint32_t lastTime = millis();

void loop() {
  webSocket.loop();

  switch (currntState) {
    case STATE_SETUP:
      {
        btSerial.println("ATSP0");
        delay(1000);
        currntState = STATE_TRANSMITTING;
      }
    case STATE_TRANSMITTING:
      {
        // if (millis() - lastTime > 2000) {
        //   lastTime = millis();
        //   btSerial.println("ATZ");
        //   delay(50);
        //   readELM("ATZ");
        // }
        btSerial.println("010c");
        readELM("010c");

        btSerial.println("0111");
        readELM("0111");

        break;
      }
    case STATE_DEBUG:
      {
        if(readLock == 0){
          readLock++;
          readELM(input);
        }
        break;
      }
  }
}

void readELM(String key) {
  while (btSerial.available() > 0) {
    char c = btSerial.read();
    if (c == '>') {
      sendData(key, data);
      data = "";
      lock--;
      break;
    }
    data += c;
  }
}

void sendData(String key, String data) {
  doc.clear();
  doc["debug"] = currntState == STATE_DEBUG;
  doc[key] = data;
  String jsonData;
  serializeJson(doc, jsonData);
  webSocket.sendTXT(jsonData);
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
        return;
      } else if (msg == "debug off") {
        currntState = STATE_SETUP;
        return;
      }

      if(currntState != STATE_DEBUG){
        return;
      }

      Serial.println(msg);
      if (!btSendLock && lock == 0) {
        btSendLock = true;
        lock++;
        input = msg;
        btSerial.println(msg);
        delay(50);
        readLock--;

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
