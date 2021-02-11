#include "esp32-hal-cpu.h"

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

const long displayOnLifetime = 5 * 1000 * 60; //5 min
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

// FUNCTIONS ------------------------------------------------------------------------------------

//MOTOR 1 CLOCKWISE
void CW1(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper1.setCurrentPosition(0);
  digitalWrite(BLUE_LED, HIGH);
  while (stepper1.currentPosition() != StepsRequired)
  {
    stepper1.setSpeed(1000);
    stepper1.runSpeed();
  }
  digitalWrite(BLUE_LED, LOW);

  stepper1.disableOutputs();
}

//MOTOR 1 COUNTER CLOCKWISE
void CCW1(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper1.setCurrentPosition(0);
  digitalWrite(BLUE_LED, HIGH);
  while (stepper1.currentPosition() != StepsRequired)
  {
    stepper1.setSpeed(1000);
    stepper1.runSpeed();
  }
  digitalWrite(BLUE_LED, LOW);

  stepper1.disableOutputs();
}

//MOTOR 2 CLOCKWISE
void CW2(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper2.setCurrentPosition(0);
  digitalWrite(BLUE_LED, HIGH);
  while (stepper2.currentPosition() != StepsRequired)
  {
    stepper2.setSpeed(1000);
    stepper2.runSpeed();
  }
  digitalWrite(BLUE_LED, LOW);

  stepper2.disableOutputs();
}

//MOTOR 2 COUNTER CLOCKWISE
void CCW2(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper2.setCurrentPosition(0);
  digitalWrite(BLUE_LED, HIGH);
  while (stepper2.currentPosition() != StepsRequired)
  {
    stepper2.setSpeed(1000);
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

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("OCLab LoRaMirror v1.0"));

    //WiFi details
    display.setCursor(0, 10);
    display.print(wifiStatus);

    //LoRa details
    display.setCursor(0, 20);
    display.print(loraStatus);

    display.setCursor(0, 30);
    display.print(loraStatus2);

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

  uint8_t DIR = payload[0];
  uint8_t STEPS = payload[1];
  uint8_t MUL = payload[2];

  if (DIR == 1) //MOTOR 1
  {
    CW1(STEPS, MUL);
  }
  if (DIR == 2) //MOTOR 1
  {
    CCW1(STEPS, MUL);
  }
  if (DIR == 3) //MOTOR 2
  {
    CW2(STEPS, MUL);
  }
  if (DIR == 4) //MOTOR 2
  {
    CCW2(STEPS, MUL);
  }

  StartFastHeartbeat();
}

void setup()
{
  setCpuFrequencyMhz(80);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("OCLab LoRaMirror");

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
}

void StartFastHeartbeat() {
  fastHeartbeatUntil = millis() + fastHeartbeatDwell;
}

void SendHeartbeat() {
  static unsigned long nextHeartbeat;

  if (millis() >= nextHeartbeat) {
    FlashBlueLED(100);
    //ttn.sendBytes(nullp, 1);
    ttn.poll();
    totalMessages++;

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
    }

    //Update OLED
    UpdateDisplay(false);
  }

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }

  SendHeartbeat();

  if (ttn.isJoined() == false) {
    delay(60000); //wait 60s then restart
    ESP.restart();
  }
}
