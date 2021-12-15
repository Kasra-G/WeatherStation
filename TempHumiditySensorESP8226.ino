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
#include "SparkFunLSM6DSO.h"

const float initialAlt = 997.0 / 3.28084;      //initial altitude in feet

//#define CITY "Marietta"
//#define STATE "Georgia"
//#define WEATHERKEY "680d20c5abcc24a2f56b6359b1b8ecb6"

const char* ssid = "HR-GH";
const char* password = "SalamHamidAgha";
const char* GScriptId = "AKfycbySwngijeWqIgPzFrUneADQOJieXu-HDXb7-2bc_ez-UjKYvptgh7vsP6N5edbF1mBvWg";
const char* host = "script.google.com";
//String weatherURL = String("http://api.openweathermap.org/data/2.5/weather?q=") + CITY + "," + STATE + "&appid=" + WEATHERKEY;

const int httpsPort = 443;
String url = String("/macros/s/") + GScriptId + "/exec?";

float dataT, dataH, dataP, dataA, dataAc;
float rdataT, rdataH, rdataP, lastdataP, rdataA, rdataAc;
double baseline = 1013.15;       //average sea level pressure of the world
const bool plot = false;
bool stopFlag = false;

void sendData();
void updateData();

HTTPSRedirect gScriptClient(httpsPort);
AsyncWebServer server(80);
Adafruit_Si7021 si7021;
Adafruit_BMP280 bmp280;
LSM6DSO lsm6dso;
TickTwo sender(sendData, 30000, 0, MILLIS);
TickTwo updater(updateData, 20, 0, MILLIS);

//Kalman filters
SimpleKalmanFilter tKalman(1, 1, 0.05);
SimpleKalmanFilter hKalman(1, 1, 0.03);
SimpleKalmanFilter pKalman(1, 1, 0.01);
SimpleKalmanFilter accKalman(1, 1, 0.10);

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

//B = sea level pressure in hPa, P = pressure in Pa, T = temp in Farenheight
//returns alt in feet
float altFromPressure(double B, float P, float T) {
  return (pow(B / (P / 100), 1 / 5.257) - 1) * ((T - 32) * 5 / 9 + 273.15) / .0065 * 3.28084;
}

//update the raw and kalman estimated data
void updateData() {
  static int count = 0;
  static bool runn = true;
  rdataT = readTempFarenh();
  rdataH = si7021.readHumidity();
  rdataP = bmp280.readPressure();
  float x, y, z;
  x = lsm6dso.readFloatAccelX();
  y = lsm6dso.readFloatAccelY();
  z = lsm6dso.readFloatAccelZ();
  float comb = sqrt(x*x +  y*y + z*z);
  
//kalman filters
  double combA = accKalman.updateEstimate(comb);
  dataT = tKalman.updateEstimate(rdataT);
  dataH = hKalman.updateEstimate(rdataH);
  lastdataP = dataP;
  dataP = pKalman.updateEstimate(rdataP);
  if (combA >= .99 && combA <= 1.01) {
    if (runn && count > 10) {
      float A = initialAlt;
      baseline = (dataP / 100) * pow(1 - .0065*A / ((dataT - 32) * 5 / 9 + .0065*A + 273.15), -5.257);
      runn = false;
    } else {
      double ratio = ((double)dataP) / lastdataP;
      baseline *= ratio;
    }
  }
  dataA = altFromPressure(baseline, dataP, dataT);

//serial plotting
  if (plot) {
    Serial.print("rdataA:");
    Serial.print(rdataA);
    Serial.print(",dataA:");
    Serial.println(dataA);
  }

  count++;
}

void sendData() {
  postData("WeatherStationReadings", dataT, dataH, dataP, dataA);
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
    WebSerial.println("Stopped Service");
  } else if (command.equals("resume") && stopFlag) {
    stopFlag = false;
    sender.resume();
    updater.resume();
    WebSerial.println("Resumed Service");
  } else if (command.equals("reboot")) {
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
  if(lsm6dso.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }
  if(lsm6dso.initialize(BASIC_SETTINGS) ) {
    Serial.println("Loaded Settings.");
  }
  sender.start();
  updater.start();
}

void loop(void) {
  AsyncElegantOTA.loop();
  sender.update();
  updater.update();
}
