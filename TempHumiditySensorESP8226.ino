#include <Wire.h>
#include <String.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <FS.h>
#include <WebSerial.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

#include "TickTwo.h"
#include "SimpleKalmanFilter.h"
#include "HTTPSRedirect.h"
#include "Adafruit_Si7021.h"
#include "Adafruit_BMP280.h"

#define CITY "Marietta"
#define STATE "Georgia"
#define WEATHERKEY "680d20c5abcc24a2f56b6359b1b8ecb6"

const char* ssid = "HR-GH";
const char* password = "SalamHamidAgha";
const char* GScriptId = "AKfycbySwngijeWqIgPzFrUneADQOJieXu-HDXb7-2bc_ez-UjKYvptgh7vsP6N5edbF1mBvWg";
const char* host = "script.google.com";
String weatherURL = String("http://api.openweathermap.org/data/2.5/weather?q=") + CITY + "," + STATE + "&appid=" + WEATHERKEY;

const int httpsPort = 443;
String url = String("/macros/s/") + GScriptId + "/exec?";

float dataT, dataH, dataP, dataA;
float rdataT, rdataH, rdataP, rdataA;
float baseline = 1017.15;
const bool plot = false;
bool stopFlag = false;

void sendData();
void updateData();
void rebase();

HTTPSRedirect gScriptClient(httpsPort);
AsyncWebServer server(80);
Adafruit_Si7021 si7021;
Adafruit_BMP280 bmp280;
TickTwo sender(sendData, 30000, 0, MILLIS);
TickTwo updater(updateData, 20, 0, MILLIS);
TickTwo baseliner(rebase, 5000, 0, MILLIS);

//Kalman filters
SimpleKalmanFilter tKalman(1, 1, 0.05);
SimpleKalmanFilter hKalman(1, 1, 0.03);
SimpleKalmanFilter pKalman(1, 1, 0.01);
SimpleKalmanFilter aKalman(1, 1, 0.10);

//Read temperature by average the barometer temperature and the hygrometer temperature
float readTempFarenh() {
  float t1 = si7021.readTemperature();
  float t2 = bmp280.readTemperature();
  return ((t1 + t2) * 9 / 10) + 32;
}

//send data to the google sheets api via POST HTTP request
void postData(String tag, float valueT, float valueH, float valueP, float valueA) {
  if (!gScriptClient.connected()) {
    WebSerial.println("Connecting to gScriptClient again…");
    gScriptClient.connect(host, httpsPort);
  }

  WebSerial.print("Sent (");
  WebSerial.print(tag);
  WebSerial.println(") to remote data logging!");

  String payload = "tag=" + tag + "&valueT=" + String(valueT) + "&valueH=" + String(valueH) + "&valueP=" + String(valueP) + "&valueA=" + String(valueA);
  gScriptClient.POST(url, host, payload);
}

//update the raw and kalman estimated data
void updateData() {
  rdataT = readTempFarenh();
  rdataH = si7021.readHumidity();
  rdataP = bmp280.readPressure();
  rdataA = bmp280.readAltitude(baseline) * 3.28084;     //convert to feet

//kalman filters
  dataT = tKalman.updateEstimate(rdataT);
  dataH = hKalman.updateEstimate(rdataH);
  dataP = pKalman.updateEstimate(rdataP);
  dataA = aKalman.updateEstimate(rdataA);
  
//serial plotting
  if (plot) {
    Serial.print("rdataA:");
    Serial.print(rdataA);
    Serial.print(",dataA:");
    Serial.println(dataA);
  }
}

void sendData() {
  postData("WeatherStationReadings", dataT, dataH, dataP, dataA);
}

void rebase() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    http.begin(client, weatherURL);
    int code = http.GET();
    if (code > 0) {
      String response = http.getString();
      if (code != 200) {
        String errorMessage = "Error response (" + String(code) + "): " + response;
        WebSerial.println(errorMessage);
      } else {
        StaticJsonDocument<768> doc;
        DeserializationError error = deserializeJson(doc, response);
        baseline = (float)(doc["main"]["pressure"]);
      }
    }
    http.end();
  }
}

String formatStats() {
  return String(dataT) + " " + String(dataH) + " " + String(dataP) + " " + String(dataA);
}

bool handleFlags(AsyncWebServerRequest* request) {
  if (stopFlag) {
    request->send(503, "text/plain", "Website is under maintenance");
    return true;
  }
  return false;
}

void parseCommand(unsigned char* data, size_t len) {
  char* d = (char*)(data);
  String command = String(d);
  command.trim();
  if (!command.startsWith("code:")) {
    return;
  }

  command.replace("code:", "");
  
  if (command.equals("stop") && !stopFlag) {
    stopFlag = true;
    sender.pause();
    updater.pause();
    baseliner.pause();
    WebSerial.println("Stopped Service");
  } else if (command.equals("resume") && stopFlag) {
    stopFlag = false;
    sender.resume();
    updater.resume();
    baseliner.resume();
    WebSerial.println("Resumed Service");
  } else if (command.equals("reboot") {
    WebSerial.println("Rebooting...");
    ESP.restart();
  }
}

void setup(void) {
  Wire.begin();
  Serial.begin(74880);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!handleFlags(request)) {
      request->send(SPIFFS, "/weather/monitor.html");
    }
  });
  server.on("/graphs", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!handleFlags(request)) {
      request->send(SPIFFS, "/weather/graphs.html");
    }
  });
  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!handleFlags(request)) {
      request->send_P(200, "text/plain", formatStats().c_str());
    }
  });
  
  WebSerial.msgCallback(parseCommand);
  AsyncElegantOTA.begin(&server, "kg87769", "FinalProj2002");
  WebSerial.begin(&server);
  server.begin();
  Serial.println("HTTP server started");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  WebSerial.print("Connected to ");
  WebSerial.println(ssid);
  WebSerial.print("IP address: ");
  WebSerial.println(WiFi.localIP().toString().c_str());
  
  WebSerial.print(String("Connecting to "));
  WebSerial.println(host);
  gScriptClient.setInsecure();
  gScriptClient.setPrintResponseBody(true);
  bool flag = false;
  for (int i = 0; i < 5; i++) {
    int retval = gScriptClient.connect(host, httpsPort);
    if (retval == 1) {
      flag = true;
      break;
    }
    else
      Serial.println("Connection failed. Retrying…");
  }
  
  Serial.println("Connection Status: " + String(gScriptClient.connected()));
  if (!flag) {
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    Serial.println("Exiting…");
    Serial.flush();
    return;
  }

  if (!si7021.begin()) {
    Serial.println("Did not find the Si7021 sensor!");
  }
  if (!bmp280.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
  }

  sender.start();
  updater.start();
  baseliner.start();
}

void loop(void) {
  AsyncElegantOTA.loop();
  sender.update();
  updater.update();
  baseliner.update();
}
