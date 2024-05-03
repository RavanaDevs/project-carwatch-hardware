#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define WIFI_SSID "sanju-server"
#define WIFI_PASSWORD "12345678"
#define WIFI_ATTEMPT_COUNT 10

#define CMD_COUNT 3

String commands[CMD_COUNT] = { "010c", "0111", "0105" };
int cmdId = 0;


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


String data = "";
String input = "";
bool reading = false;
static uint32_t lastTime = millis();

void loop() {
  webSocket.loop();

  if (millis() - lastTime > 1000) {
    lastTime = millis();

    if (!reading) {
      btSerial.println(commands[cmdId]);
      delay(100);
      cmdId++;
      if (cmdId == CMD_COUNT)
        cmdId = 0;
    }

    while (btSerial.available() > 0) {
      reading = true;
      char c = btSerial.read();
      if (c == '>') {
        sendData("data",data);
        data = "";
        reading = false;
        break;
      }
      data += c;
    }
    // delay(1000);
  }
}


void sendData(String key, String data) {
  doc.clear();
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
