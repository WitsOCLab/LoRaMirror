#include <Arduino.h>
#include <math.h>

// General --------------------------------------------------------------------
#include "AsyncTimerLib.h"
#include <DebounceEvent.h>

// Pins -----------------------------------------------------------------------
#define OLED_RST      19
#define OLED_SDA      20
#define OLED_SCL      21

#define JOY_BTN       10
#define JOY_LEFT      11
#define JOY_RIGHT     12
#define JOY_UP        13
#define JOY_DOWN      14


// Display --------------------------------------------------------------------
#include <U8g2lib.h>
#include <Wire.h>
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_RST);
const unsigned long SCREEN_UPDATE_MS = 50; // 1000/FPS

// Mirror Visualisation
const u8g2_uint_t MV_WIDTH = 64;
const u8g2_uint_t MV_HEIGHT = 64;
const u8g2_uint_t MV_X = 128 - MV_WIDTH - 0; //A border of 7px all around
const u8g2_uint_t MV_Y = 0;

const float MV_HEIGHT_OFFSET = MV_Y + (float)MV_HEIGHT / 2;
const float MV_HEIGHT_PX_PERCENT = (float)MV_HEIGHT / 2 / 100.0;
const float MV_WIDTH_OFFSET = MV_X + (float)MV_WIDTH / 2;
const float MV_WIDTH_PX_PERCENT = (float)MV_WIDTH / 2 / 100.0;


// State ----------------------------------------------------------------------
float _mirrorXTilt = 0; //% [+- 100]
float _mirrorYTilt = 0; //% [+- 100]

bool _moveFast = false;


struct CommandStruct {
  uint16_t moveX; //bias 32767 = 0
  uint16_t moveY; //bias 32767 = 0
} _nextCmd;

struct TelemetryStruct {
  uint16_t percentX; //bias 32767 = 0, 0= -100%, 65535=+100%
  uint16_t percentY; //bias 32767 = 0, 0= -100%, 65535=+100%
    uint8_t temperature;
} _lastTele;

// Timed Functions ------------------------------------------------------------------

void LEDToggle() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
AsyncTimer LEDTimer(1000000, true, LEDToggle);

void UpdateScreen() {
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_ncenB08_tr);


  // Draw mirror visualisation
  u8g2.drawFrame(MV_X, MV_Y, MV_WIDTH, MV_HEIGHT);
  u8g2.drawHLine(MV_X, (u8g2_uint_t)(round(MV_HEIGHT_OFFSET + MV_HEIGHT_PX_PERCENT * _mirrorYTilt)), MV_WIDTH);
  u8g2.drawVLine((u8g2_uint_t)(round(MV_WIDTH_OFFSET + MV_WIDTH_PX_PERCENT * _mirrorXTilt)), MV_Y, MV_HEIGHT);

  u8g2.drawBox(0, 0, 128 - MV_WIDTH, 12);
  u8g2.setDrawColor(0);
  if (_moveFast) {
    u8g2.drawStr((128 - MV_WIDTH) / 2 - u8g2.getStrWidth("FAST") / 2, 10, "FAST");
  } else {
    u8g2.drawStr((128 - MV_WIDTH) / 2 - u8g2.getStrWidth("SLOW") / 2, 10, "SLOW");
  }

  u8g2.sendBuffer();

  static int mvx = 3;
  static int mvy = 3;
  _mirrorYTilt += mvy;
  if (abs(_mirrorYTilt) >= 100) mvy = - mvy;
  _mirrorXTilt += mvx;
  if (abs(_mirrorXTilt) >= 100) mvx = - mvx;
}
AsyncTimer displayTimer(SCREEN_UPDATE_MS * 1000, true, UpdateScreen);

// Input Functions --------------------------------------------------------------------

void JoyPress(uint8_t pin, uint8_t event, uint8_t count, uint16_t length) {
  if ((event == EVENT_PRESSED) && (count == 1)) {
    _moveFast = !_moveFast;
  }

  // Debug
  Serial.print("Event : "); Serial.print(event);
  Serial.print(" Count : "); Serial.print(count);
  Serial.print(" Length: "); Serial.print(length);
  Serial.println();
}
DebounceEvent _joyBtn = DebounceEvent(JOY_BTN, JoyPress, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);

void JoyRight(uint8_t pin, uint8_t event, uint8_t count, uint16_t length) {
  if (_moveFast) {
    
  } else {
    
  }
}
DebounceEvent _joyRight = DebounceEvent(JOY_RIGHT, JoyRight, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);


// Setup and Loop ------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);

  LEDTimer.Start();
  displayTimer.Start();

  u8g2.begin();
}


void loop() {
  // Update timed things
  LEDTimer.Update();
  displayTimer.Update();

  // Update inputs
  _joyBtn.loop();
}
