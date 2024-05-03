#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define WIFI_SSID "sanju-server"
#define WIFI_PASSWORD "12345678"
#define WIFI_ATTEMPT_COUNT 10

#define CMD_COUNT 3


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
static uint32_t lastTime = millis();

void loop() {
  webSocket.loop();


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
