/*
   Includes:
   AccelStepper, TTN_esp32, Adafruit SSD1306
   Use GIT: https://github.com/rgot-org/TheThingsNetwork_esp32
*/

#include <math.h>
#include "esp32-hal-cpu.h"

const String strName = F("OCLab LoRaMirror v2.2");

// TTN ------------------------------------------------------------------------------------------

#include <TTN_esp32.h>

#include "TTN_CayenneLPP.h"

const char* devEui = "002CF661A596CF8E"; // Change to TTN Device EUI
const char* appEui = "70B3D57ED003D602"; // Change to TTN Application EUI
const char* appKey = "4479DE992478DB2190FD20D99201AF37"; // Chaneg to TTN Application Key

TTN_esp32 ttn ;
TTN_CayenneLPP lpp;

#define UNUSED_PIN 0xFF
#define SS 18
#define RST_LoRa 14
#define DIO0 26
#define DIO1 33
#define DIO2 32

unsigned long totalMessages = 0;

unsigned long heartbeatInterval = 10000;
unsigned long fastHeartbeatInterval = 5000;
unsigned long fastHeartbeatUntil = 0;
unsigned long fastHeartbeatDwell = 30000;

unsigned int ackEveryN = 100;
const unsigned int notJoinedCounterLimit = 10;
unsigned int notJoinedCounter = 0;

// WIFI -----------------------------------------------------------------------------------------

#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* host = "LoRaMirror";
const char* ssid = "VectorMode";
const char* password = "VectorMode123";


// OLED -----------------------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

unsigned long displayOnLifetime = 5 * 1000 * 60; //10 min
long displayOffMillis;
bool displayOn = false;

String wifiStatus = "WiFi Connecting...";
String loraStatus = "";
String loraStatus2 = "";

const int BLUE_LED = 17;

// STEPPERS -------------------------------------------------------------------------------------

#include <AccelStepper.h>
// Number of steps per internal motor revolution
const float STEPS_PER_REV = 32;

//  Amount of Gear Reduction
const float GEAR_RED = 64;

// Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;

int StepsRequired;
#define HALFSTEP 8 //FULLSTEP = 8

/*STEPPER PINS
  1: 13, 12, 21, 25 ----> IN1, IN2, IN3, IN4

  2: 22, 23, 0, 2 ----> IN1, IN2, IN3, IN4
*/
AccelStepper stepper1 = AccelStepper(HALFSTEP, 13, 21, 12, 25);
AccelStepper stepper2 = AccelStepper(HALFSTEP, 22, 0, 23, 2);

const float mradPerTurn = 8; //get from datasheet

int lastStepsX, lastStepsY;

// GPS ------------------------------------------------------------------------------------------

// GPS will do it's thing while the screen is on. If it can't find signal by then, sorry...

#include <TinyGPS++.h>

//GPS RX ---> PIN 35 ESP
//GPS TX ---> PIN 34 ESP
#define RXD2 34
#define TXD2 35

TinyGPSPlus gps;

struct  GpsDataState_t {
  double originLat = 0 ;
  double originLon = 0 ;
  double originAlt = 0 ;
  double distMax = 0 ;
  double dist = 0 ;
  double altMax = - 999999 ;
  double altMin = 999999 ;
  double spdMax = 0 ;
  double prevDist = 0 ;
};
GpsDataState_t gpsState = {};

const double LAB_LAT = -26.1913173;
const double LAB_LON = 28.0268707;

// FUNCTIONS ------------------------------------------------------------------------------------

int mradToSteps(float mrad) {
  return (int)((mrad / mradPerTurn) * STEPS_PER_OUT_REV);
}

int resultantPosToSteps(float resultantMeters, float mirrorDistance) {
  float mrad = atan2(resultantMeters, mirrorDistance) * 1000;
  return mradToSteps(mrad);
}

void MoveStepperOne(int StepsRequired)
{
  lastStepsX = StepsRequired;
  digitalWrite(BLUE_LED, HIGH);
  stepper1.setCurrentPosition(0);

  if (StepsRequired > 0) {
    stepper1.setSpeed(1000);
  } else {
    stepper1.setSpeed(-1000);
  }

  while (stepper1.currentPosition() != StepsRequired)
  {
    stepper1.runSpeed();
  }
  digitalWrite(BLUE_LED, LOW);

  stepper1.disableOutputs();
}

void MoveStepperTwo(int StepsRequired)
{
  lastStepsY = StepsRequired;
  digitalWrite(BLUE_LED, HIGH);
  stepper2.setCurrentPosition(0);

  if (StepsRequired > 0) {
    stepper2.setSpeed(1000);
  } else {
    stepper2.setSpeed(-1000);
  }

  while (stepper2.currentPosition() != StepsRequired)
  {
    stepper2.runSpeed();
  }
  digitalWrite(BLUE_LED, LOW);

  stepper2.disableOutputs();
}


void ToggleBlueLED() {
  digitalWrite(BLUE_LED, !digitalRead(BLUE_LED));
}

void FlickerBlueLED() {
  static unsigned long nextFlicker;
  static int longBreak = 0; //if nextFlicker should be a while

  if (millis() >= nextFlicker) {
    switch (longBreak) {
      case 0: //on, short wait
        digitalWrite(BLUE_LED, HIGH);
        nextFlicker = millis() + 100;
        longBreak++;
        break;
      case 1: //off, short wait
        digitalWrite(BLUE_LED, LOW);
        nextFlicker = millis() + 100;
        longBreak++;
        break;
      case 2: //on, short wait
        digitalWrite(BLUE_LED, HIGH);
        nextFlicker = millis() + 100;
        longBreak++;
        break;
      case 3: //off, long wait
        digitalWrite(BLUE_LED, LOW);
        nextFlicker = millis() + 5000;
        longBreak = 0;
        break;
    }
  }
}

void FlashBlueLED(unsigned int howLong = 0) {
  static unsigned long onUntil = 0;

  if (howLong > 0) {
    digitalWrite(BLUE_LED, HIGH);
    onUntil = millis() + howLong;
  } else {
    if ((onUntil > 0) && (millis() >= onUntil)) {
      digitalWrite(BLUE_LED, LOW);
      onUntil = 0;
    }
  }
}

void UpdateDisplay(bool immediate = true) {
  static unsigned long nextUpdate;

  if ((immediate) || (millis() >= nextUpdate)) {

    if (WiFi.status() == WL_CONNECTED) {
      wifiStatus = "WiFi: " + WiFi.localIP().toString();
    } else {
      wifiStatus = "WiFi Disconnected";
    }

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(strName);

    //WiFi details
    display.setCursor(0, 10);
    display.print(wifiStatus);

    //LoRa details
    display.setCursor(0, 20);
    display.print(loraStatus);

    display.setCursor(0, 30);
    display.print(loraStatus2);

    display.setCursor(0, 40);
    display.print("Last: (" + String(lastStepsX) + "," + String(lastStepsY) + ")");

    if (gps.location.isValid()) {
      display.setCursor(0, 50);
      display.print("GPS: " + String(gps.location.lat()) + ", " + String(gps.location.lng()));
    } else {
      display.setCursor(0, 50);
      char sz[32];
      sprintf(sz, "%02d:%02d:%02d ", gps.time.hour(), gps.time.minute(), gps.time.second());
      display.print("GPS Time: " + String(sz));
    }

    display.display();

    nextUpdate = millis() + 1000;
  }
}


void message(const uint8_t* payload, size_t size, int rssi)
{
  //  Serial.println("-- MESSAGE");
  //  Serial.print("Received " + String(size) + " bytes RSSI=" + String(rssi) + "db");
  //  for (int i = 0; i < size; i++)
  //  {
  //    Serial.print(" " + String(payload[i]));
  // Serial.write(payload[i]);
  //  }

  loraStatus2 = "Last RSSI: " + String(rssi) + " dB";

  uint8_t command = payload[0]; //command

  /* TTN steps encoder:
    var myVal = 20000;
    var bytes = [];
    bytes[0] = (myVal & 0xFF00) >> 8;
    bytes[1] = (myVal & 0x00FF);
  */
  uint16_t steps;
  int16_t resultantmm;
  uint16_t distancecm;

 // Serial.println(String(payload[0],HEX) + " " + String(payload[1],HEX));

  switch (command) {
    case 1: //tip steps
      steps = ((uint16_t)(payload[2]) << 8) + (uint16_t)payload[1];
      MoveStepperOne(steps);
      break;
    case 2: //-tip steps
      steps = ((unsigned int)(payload[2]) << 8) + payload[1];
      MoveStepperOne(-steps);
      break;
    case 3: //tilt steps
      steps = ((unsigned int)(payload[2]) << 8) + payload[1];
      MoveStepperTwo(steps);
      break;
    case 4: //-tilt steps
      steps = ((unsigned int)(payload[2]) << 8) + payload[1];
      MoveStepperTwo(-steps);
      break;
    case 5: //tip distance
      resultantmm = ((int)(payload[2]) << 8) + payload[1];
      distancecm = ((int)(payload[2]) << 8) + payload[1];
      MoveStepperOne(resultantPosToSteps((float)resultantmm / 1000.0, (float)distancecm / 100.0));
      break;
    case 6: //tilt distance
      resultantmm = ((int)(payload[2]) << 8) + payload[1];
      distancecm = ((int)(payload[2]) << 8) + payload[1];
      MoveStepperTwo(resultantPosToSteps((float)resultantmm / 1000.0, (float)distancecm / 100.0));
      break;
  }

  StartFastHeartbeat();
}

void setup()
{
  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  delay(1000);
  Serial.println(strName);

  pinMode(BLUE_LED, OUTPUT);

  //OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    displayOn = false;
  } else {
    wifiStatus = "WiFi Connecting";
    UpdateDisplay();

    displayOffMillis = millis() + displayOnLifetime;
    Serial.println("millis() now: " + String(millis()) + ". Display will turn off at " + String(displayOffMillis));
    displayOn = true;
  }

  //Wifi
  btStop(); //turn off bluetooth
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print(F("Trying WiFi connection"));

  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);

  // Wait for connection
  int wifiWait = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    wifiWait += 500;

    wifiStatus = wifiStatus + ".";
    UpdateDisplay();
    ToggleBlueLED();

    if (wifiWait >= 5000) { //give up wifi
      Serial.println(F(" Give up!"));
      wifiStatus = "WiFi Disabled";
      UpdateDisplay();

      WiFi.mode(WIFI_OFF); // put wifi to sleep to save power
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    displayOnLifetime = 60 * 1000 * 1000; //1h
    displayOffMillis = millis() + displayOnLifetime; //update

    Serial.println();
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    wifiStatus = "WiFi: " + WiFi.localIP().toString();
    UpdateDisplay();

    // setup OTA stuff
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
  } else {
    //run nice and slow
    setCpuFrequencyMhz(20);
  }

  UpdateDisplay();

  //TTN
  ttn.begin(SS, UNUSED_PIN, RST_LoRa, DIO0, DIO1, DIO2);
  ttn.onMessage(message); // Declare callback function for handling downlink
  // messages from server
  ttn.join(devEui, appEui, appKey);
  Serial.print("Joining TTN ");

  loraStatus = "Lora Connecting";
  UpdateDisplay();

  while (!ttn.isJoined())
  {
    loraStatus = loraStatus + ".";
    UpdateDisplay();
    ToggleBlueLED();
    Serial.print(".");
    delay(500);
  }

  Serial.println("\njoined !");
  loraStatus = "LoRa Connected";
  UpdateDisplay();
  digitalWrite(BLUE_LED, LOW); //LED off means all good.
  ttn.showStatus();

  //GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

}

void StartFastHeartbeat() {
  fastHeartbeatUntil = millis() + fastHeartbeatDwell;
}

void SendHeartbeat() {
  static unsigned long nextHeartbeat;

  if (millis() >= nextHeartbeat) {
    FlashBlueLED(100);
    //ttn.sendBytes(nullp, 1);
    totalMessages++;

    ttn.poll(1, (totalMessages % ackEveryN) == 0);

    if (fastHeartbeatUntil == 0) {
      nextHeartbeat = millis() + heartbeatInterval;
    } else {
      nextHeartbeat = millis() + fastHeartbeatInterval;
      if (millis() >= fastHeartbeatUntil) {
        fastHeartbeatUntil = 0; //stop fast mode
      }
    }
  }
}

void ProcessGPS() {
  // If we get there, GPS has some valid data.
  if (gps.location.isValid()) {
    Serial.println("GPS Location: " + String(gps.location.lat()) + ", " + String(gps.location.lng()));
    Serial.println("Altitude: " + String(gps.altitude.meters()) + " m");

    double distanceToLabMeters = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), LAB_LAT, LAB_LON);
    Serial.println("Distance to lab: " + String(distanceToLabMeters) + " m");

    double courseToLab = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), LAB_LAT, LAB_LON);
    Serial.println("Course to lab: " + String(courseToLab) + "°N");
  }
}

void loop()
{
  // FlickerBlueLED();
  FlashBlueLED(); //just an update

  if (displayOn) {
    loraStatus = "LoRa Messages: " + String(totalMessages);

    if (millis() >= displayOffMillis) {
      displayOn = false;
      display.clearDisplay();
      display.ssd1306_command(SSD1306_DISPLAYOFF);

      //also turn off wifi

      if (WiFi.status() == WL_CONNECTED) {
        WiFi.mode(WIFI_OFF);
      }
    }

    //GPS
    while (Serial2.available() > 0)
    {
      char r = Serial2.read();
      Serial.print(r);
      if (gps.encode(r)) {
        ProcessGPS();
      }
    }

    //Update OLED
    UpdateDisplay(false);
  }


  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }

  SendHeartbeat();

  if (ttn.isJoined() == false) {
    notJoinedCounter++;
    delay(5000);
    if (notJoinedCounter >= notJoinedCounterLimit) {
      ESP.restart();
    }
  } else {
    notJoinedCounter = 0;
  }
}
