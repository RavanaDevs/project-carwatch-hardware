#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <floatToString.h>
#include <TinyGPSPlus.h>

// #define WIFI_SSID "Sanju's iPhone"
#define WIFI_SSID "sanju-server"
#define WIFI_PASSWORD "12345678"
//#define WIFI_ATTEMPT_COUNT 10

#define TIMER1_SIZE 2
#define TIMER2_SIZE 2
#define TIMER3_SIZE 2

#define TIMER1_DELAY 1000
#define TIMER2_DELAY 5500
#define TIMER3_DELAY 9000
#define TIMER4_DELAY 15000

String timer1_cmd[TIMER1_SIZE] = { "010c", "010d" };
String timer2_cmd[TIMER2_SIZE] = { "0111", "0105" };  // throttle position, engine coolent temp
String timer3_cmd[TIMER3_SIZE] = { "0103", "0104" };  // fuel system status, fuel pressure

int timer1Id = 0;
int timer2Id = 0;
int timer3Id = 0;

uint32_t timer1 = millis();
uint32_t timer2 = millis();
uint32_t timer3 = millis();
uint32_t timer4 = millis();

TinyGPSPlus gps;
SoftwareSerial gpsSerial(D6, D5);
SoftwareSerial btSerial(D4, D3);  //TX RX
WebSocketsClient webSocket;
WiFiClient wifiClient;
StaticJsonDocument<1024> doc;


void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  delay(5000);

  pinMode(D2, INPUT_PULLUP);

  Serial.println("Connecting");
  connectToWIFI();
  Serial.println("\nConnected");
  Serial.println(WiFi.localIP());
  //local -> 172.20.10.5
  //int -> 172.105.34.106
  webSocket.begin("172.20.10.5", 5000, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}


String data = "";
String input = "";
bool reading = false;
static uint32_t lastTime = millis();

bool printed = false;
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void loop() {
  webSocket.loop();

  int reading = digitalRead(D2);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the current reading is different from the button state
    if (reading != buttonState) {
      buttonState = reading;

      // If the button is pressed and the string hasn't been printed yet
      if (buttonState == LOW && !printed) {
        // Print the string to the serial port
        sendData("hzd", "Hazzard alert");
        Serial.println("Sendign HZD");

        // Set the flag to true to indicate that the string has been printed
        printed = true;
      } else if (buttonState == LOW) {
        // If the button is not pressed, reset the flag
        printed = false;
      }
    }
  }

  // Update last button state
  lastButtonState = reading;


  uint32_t currntTime = millis();

  if ((currntTime - timer1) > TIMER1_DELAY) {
    timer1 = currntTime;
    if (!reading) {
      btSerial.println(timer1_cmd[timer1Id]);
      delay(100);
      timer1Id++;
      if (timer1Id == TIMER1_SIZE) {
        timer1Id = 0;
      }
    }
  }

  if ((currntTime - timer2) > TIMER2_DELAY) {
    timer2 = currntTime;
    if (!reading) {
      btSerial.println(timer2_cmd[timer2Id]);
      delay(100);
      timer2Id++;
      if (timer2Id == TIMER2_SIZE) {
        timer2Id = 0;
      }
    }
  }

  if ((currntTime - timer3) > TIMER3_DELAY) {
    timer3 = currntTime;
    if (!reading) {
      btSerial.println(timer3_cmd[timer3Id]);
      delay(100);
      timer3Id++;
      if (timer3Id == TIMER3_SIZE) {
        timer3Id = 0;
      }
    }
  }

  if ((currntTime - timer4) > TIMER4_DELAY) {
    timer4 = currntTime;
    getLocation();
  }

  while (btSerial.available() > 0) {
    reading = true;
    char c = btSerial.read();
    if (c == '>') {
      sendData("data", data);
      data = "";
      reading = false;
      break;
    }
    data += c;
  }
}

String LON, LAT;

void getLocation() {
  LAT = String(gps.location.lat(), 6);
  LON = String(gps.location.lng(), 6);

  String location = LON + "," + LAT;
  // Serial.println(location);
  sendData("gps", location);

  encordGPS(700);
}

void encordGPS(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
  } while (millis() - start < ms);
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
