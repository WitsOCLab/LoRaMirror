/*  
 * This code is to be used with an ESP32 Built-in OLED Lora TTGO Lora32 board.
 * Arduino IDE board type - TTGO LoRa32-OLED V1
 */

/*
   THE THINGS NETWORK THINGS
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

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

//***************************************************

/*
   OTAA WEB UPDATER THINGS
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

const char* host = "esp32";
const char* ssid = "VectorMode";
const char* password = "VectorMode123";

WebServer server(80);

/*
 * Login page
 */

const char* loginIndex = 
 "<form name='loginForm'>"
      "<table width='80%' height='30%' bgcolor='D98318' align='center'>"
        "<tr>"
            "<td colspan=8>"
                "<center><font size=8><b>OTAA WEB UPDATER</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td><font bold><b>USERNAME:<b></font></td>"
        "<td><input type='text' size=50 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td><font bold><b>PASSWORD:</b></font></td>"
            "<td><input type='Password' size=50 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='LOGIN'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='ramiz' && form.pwd.value=='ramiz')" //Change username and password here for the OTAA Web Updater page
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'><font bold><b>PROGRESS: 0%</b></font></div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";
//***************************************************



/*
   TRANSMISSION THINGS
*/
byte mydata[5]; 
static osjob_t sendjob;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 120000; //2 minute

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned TX_INTERVAL1 = 5; // 5 second intervals


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};
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

//*******************************************************************

/*
   INDICATOR LED THINGS
*/

  int BLUE_LED = 17;
  unsigned long startMillis2;
  unsigned long currentMillis2;

//*******************************************************************


void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_JOINING:
      Serial.println(F("EV_JOINING: -> Joining..."));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

      if (LMIC.txrxFlags & TXRX_ACK)
      {
        Serial.println(F("Received ack"));
      }
      if (LMIC.dataLen) {
        Serial.println(F("Data Received:"));
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

        Serial.println();
      }

      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL1), do_send);
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
      Serial.println(F("Unknown event"));
      break;
  }
}

//MOTOR 1 CLOCKWISE
void CW1(uint8_t DemSteps, uint8_t DemScale)
{
  StepsRequired = DemSteps * DemScale;
  stepper1.setCurrentPosition(0);
  while(stepper1.currentPosition() != StepsRequired)
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
  while(stepper1.currentPosition() != StepsRequired)
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
  while(stepper2.currentPosition() != StepsRequired)
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
  while(stepper2.currentPosition() != StepsRequired)
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
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Sending uplink packet..."));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


void setup() {

  Serial.begin(115200);
  //MOTOR THINGS
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
  
  //WEB UPDATER
   // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
  

  //INDICATOR LED 
  pinMode(BLUE_LED, OUTPUT);
  startMillis2 = millis();
  
  //TRANSMISSION THINGS
  startMillis = millis();
  
  //GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println(F("Starting"));

  //OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();
  delay(2000);
  display.clearDisplay();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

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

  //Start job (sending automatically starts OTAA too)
  do_send(&sendjob);

}

bool change = false;

void BlinkSlow()
{
   if(currentMillis2 - startMillis2 <=3000)
  {
     digitalWrite(BLUE_LED, HIGH);
  }

  if(currentMillis2 - startMillis2 >= 3000 && currentMillis2 - startMillis2 <= 6000)
  {
    digitalWrite(BLUE_LED, LOW);
  }
  if(currentMillis2 - startMillis2 >= 6000)
  {
    startMillis2 = currentMillis2;
  }
}

void BlinkFast()
{
   if(currentMillis2 - startMillis2 <=300)
  {
     digitalWrite(BLUE_LED, HIGH);
  }

  if(currentMillis2 - startMillis2 >= 300 && currentMillis2 - startMillis2 <= 600)
  {
    digitalWrite(BLUE_LED, LOW);
  }
  if(currentMillis2 - startMillis2 >= 600)
  {
    startMillis2 = currentMillis2;
  }
}

void loop() {

  os_runloop_once();

  //WEB UPDATER
  server.handleClient();
  delay(1);

  currentMillis = millis();
  currentMillis2 = millis();
 
  //TRANSMISSION THINGS

  if (currentMillis - startMillis >= period && (currentMillis - startMillis) <= (period +  30000))
  {
    change = true;
  }
  if (currentMillis - startMillis >= (period + 30000))
  {
    startMillis = currentMillis; //allows for repeating functionality
    change = false;
  }

  if (change == false) {
    BlinkSlow();
    
    mydata[0] = 0;
    mydata[1] = 0;
    mydata[2] = 0;
    mydata[3] = 0;
    mydata[4] = 0;
  }

  else if (change == true )
  { 
    BlinkFast();
    //GPS
    while (Serial2.available() > 0)
    {
      if (gps.encode(Serial2.read()))
        GPSInfo();
    }
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while (true);
    }

  }

}
