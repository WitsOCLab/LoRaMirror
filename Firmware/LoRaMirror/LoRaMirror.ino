/*
   This code is to be used with an ESP32 Built-in OLED Lora TTGO Lora32 board.
   Arduino IDE board type - TTGO LoRa32-OLED V1
*/

/*
   THE THINGS NETWORK THINGS
*/

//https://github.com/LilyGO/TTGO-LORA32/blob/master/OLED_LoRa_Receive/OLED_LoRa_Receive.ino

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define CFG_eu868 1
#define CFG_sx1276_radio 1

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)
#define BAND 868E6 // 915E6

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = SS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RST,
  .dio = {DI0, 33, 32},
};

static const u1_t PROGMEM APPEUI[8] = { 0x23, 0x92, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM DEVEUI[8] = { 0x78, 0x56, 0x34, 0x12, 0x89, 0x67, 0x45, 0x23 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPKEY[16] = { 0x76, 0x4C, 0x6D, 0x8B, 0x2C, 0x3B, 0x71, 0xE7, 0x95, 0x97, 0xD5, 0x42, 0xBF, 0x14, 0xB8, 0x68 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

bool loraConnected = false;

//***************************************************

/*
   OTAA WEB UPDATER THINGS
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* host = "loramirror";
const char* ssid = "VectorMode";
const char* password = "VectorMode123";


/*
   TRANSMISSION THINGS
*/
byte mydata[16];
int bytesToSend = 0; //0 = idle

static osjob_t sendjob;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 120000; //2 minute

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned long TX_INTERVAL_IDLE = 10; 
unsigned long TX_INTERVAL_ACTIVE = 5;

unsigned long txBytes = 0;





//*********************************************************************
/*
   STEPPER MOTOR THINGS
*/

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
//*************************************************************************

/*
  GPS THINGS
*/
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
//****************************************************************

/*
   OLED THINGS
*/

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

//*******************************************************************

/*
   INDICATOR LED THINGS
*/

int BLUE_LED = 17;

//*******************************************************************

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
  unsigned long nextTXTime = TX_INTERVAL_IDLE;
  
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      loraConnected = true;
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

      if (LMIC.txrxFlags & TXRX_ACK)
      {
        Serial.println(F("Received ack"));
      }
      if (LMIC.dataLen) {
        Serial.println(F("Data Received: "));
        Serial.println(LMIC.dataLen);
        Serial.println(F("bytes of payload"));

        uint8_t DIR = LMIC.frame[LMIC.dataBeg + 0];
        uint8_t STEPS = LMIC.frame[LMIC.dataBeg + 1];
        uint8_t MUL = LMIC.frame[LMIC.dataBeg + 2];

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

        nextTXTime = TX_INTERVAL_ACTIVE;
      }

      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(nextTXTime), do_send);
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.print(F("EV_UNKNOWN: "));
      Serial.println((unsigned) ev);
      break;
  }
}

//MOTOR 1 CLOCKWISE
void CW1(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper1.setCurrentPosition(0);
  while (stepper1.currentPosition() != StepsRequired)
  {
    stepper1.setSpeed(1000);
    stepper1.runSpeed();
    digitalWrite(BLUE_LED, HIGH);
  }

  stepper1.disableOutputs();

}

//MOTOR 1 COUNTER CLOCKWISE
void CCW1(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper1.setCurrentPosition(0);
  while (stepper1.currentPosition() != StepsRequired)
  {
    stepper1.setSpeed(1000);
    stepper1.runSpeed();
    digitalWrite(BLUE_LED, HIGH);
  }

  stepper1.disableOutputs();
}

//MOTOR 2 CLOCKWISE
void CW2(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper2.setCurrentPosition(0);
  while (stepper2.currentPosition() != StepsRequired)
  {
    stepper2.setSpeed(1000);
    stepper2.runSpeed();
    digitalWrite(BLUE_LED, HIGH);
  }

  stepper2.disableOutputs();


}

//MOTOR 2 COUNTER CLOCKWISE
void CCW2(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper2.setCurrentPosition(0);
  while (stepper2.currentPosition() != StepsRequired)
  {
    stepper2.setSpeed(1000);
    stepper2.runSpeed();
    digitalWrite(BLUE_LED, HIGH);
  }

  stepper2.disableOutputs();

}

void GPSInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    if (gps. satellites . value () > 4 ) {
      gpsState. dist = TinyGPSPlus :: distanceBetween (gps. location . lat (), gps. location . lng (), gpsState. originLat , gpsState. originLon );

      if (gpsState. dist > gpsState. distMax && abs (gpsState. prevDist - gpsState. dist ) < 50 ) {
        gpsState. distMax = gpsState. dist ;
      }
      gpsState. prevDist = gpsState. dist ;

      if (gps. altitude . meters () > gpsState. altMax ) {
        gpsState. altMax = gps. altitude . meters ();
      }

      if (gps. speed . kmph () > gpsState. spdMax ) {
        gpsState. spdMax = gps. speed . kmph ();
      }

      if (gps. altitude . meters () < gpsState. altMin ) {
        gpsState. altMin = gps. altitude . meters ();
      }
    }
    int32_t Loc_Lat = -gps.location.lat() * 100;
    uint32_t Loc_Lng = gps.location.lng() * 100;

    mydata[0] = highByte(Loc_Lat);
    mydata[1] = lowByte(Loc_Lat);
    mydata[2] = highByte(Loc_Lng);
    mydata[3] = lowByte(Loc_Lng);
    mydata[4] = 0;



    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("LAT: ");
    display.setCursor(10, 0);
    display.print(gps.location.lat(), 6);
    display.setCursor(0, 20);
    display.print("LNG: ");
    display.setCursor(10, 20);
    display.print(gps.location.lng(), 6);
    display.display();

    Serial.println();
    Serial.print ( " LAT = " ); Serial.println (gps.location.lat(), 6 );
    Serial.print ( " LONG = " ); Serial.println (gps.location.lng(), 6 );
    Serial.print ( " ALT = " ); Serial.println (gps.altitude.meters());
    Serial.print ( " Sats = " ); Serial.println (gps.satellites.value());
    Serial.print ( " DST (m): " );
    Serial.println (gpsState. dist , 1 );
  }
  else
  {
    mydata[0] = 0;
    mydata[1] = 0;
    mydata[2] = 0;
    mydata[3] = 0;
    mydata[4] = 0;
    display.clearDisplay();
    Serial.print(F("INVALID"));
    display.setCursor(0, 0);
    display.print("GPS: NO SIGNAL");
    display.display();
  }
  Serial.println();
}



void do_send(osjob_t* j) {

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else {
    if (bytesToSend == 0) {
      LMIC_setTxData2(1, 0, 1, 0);
      txBytes++;
      Serial.println(F("LoRa Idle"));
    } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, mydata, bytesToSend, 0);
      txBytes += bytesToSend;
      Serial.println(F("LoRa Packet"));
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void UpdateDisplay() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("OCLab LoRaMirror v1.0"));

  //WiFi details
  display.setCursor(0, 10);
  if (WiFi.status() != WL_CONNECTED) {
    display.print(String(ssid) + " Down");
  } else {
    display.print(WiFi.localIP().toString());
  }

  //LoRa details
  display.setCursor(0, 20);
  if (loraConnected == false) {
    display.print(F("LoRa Not Connected"));
  } else {
    display.print("LoRa Sent " + String(txBytes) + " B");
  }

  //TODO: Add GPS stuff...

  display.display();
}

void setup() {
  delay(100);
  Serial.begin(115200);

  //MOTOR THINGS
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);

  //OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    displayOn = false;
  } else {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("OCLab LoRaMirror v1.0"));
    display.setCursor(0, 10);
    display.print(F("Connecting to WiFi..."));
    //display.setCursor(0, 20);
    //display.print(F("Connecting to LoRa..."));
    display.display();

    displayOffMillis = millis() + displayOnLifetime;
    Serial.println("millis() now: " + String(millis()) + ". Display will turn off at " + String(displayOffMillis));
    displayOn = true;
  }

  //WEB UPDATER
  // Connect to WiFi network if we find one
  btStop(); //turn off bluetooth
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print(F("Trying WiFi connection"));

  // Wait for connection
  int wifiWait = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    wifiWait += 500;

    if (wifiWait >= 5000) { //give up wifi
      Serial.println(F(" Give up!"));
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
  }

  UpdateDisplay();

  //INDICATOR LED
  pinMode(BLUE_LED, OUTPUT);

  //GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);


  Serial.println("Starting LoRa...");
  // LMIC init
  os_init();
  
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  /* LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    //Disable link check validation
    LMIC_setLinkCheckMode(0);

    //TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    //Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(DR_SF9, 14);
  */
  //Start job (sending automatically starts OTAA too)
  do_send(&sendjob);

}



void loop() {

  os_runloop_once();

  if (displayOn) {
    if (millis() >= displayOffMillis) {
      displayOn = false;
      display.clearDisplay();
      display.ssd1306_command(SSD1306_DISPLAYOFF);
    }

    //Update OLED
    UpdateDisplay();
  }

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }

}
