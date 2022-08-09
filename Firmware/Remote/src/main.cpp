#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <AsyncTimer.h>
#include <Yabl.h>

#include "CommunicationProtocol.h"
#include "Joystick.h"

// Initializing objects
CommunicationProtocol comms;
Joystick stick;
AsyncTimer t;
Button button;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

// some variables
int remoteBatteryRaw = 0;
float remoteBatteryVoltage = 0;
bool zeroSent = false;
String configStrings[6]{"Sleep", "Battery", "Calibrate", "Center Stick", "Deadzone", "About"};
int menuItemsCount = 6;
int menuIndex = 0;
int aboutIndex = 0;
int aboutItemsCount = 6;
int settingVariable = 0;
bool slowMode = false;
int receivedSize = 0;
int xPos = 0;
int yPos = 0;
float mirrorBattery = 0;
int heartbeatReceiveTimeout = 0;
bool heartbeatReceived = true;

enum class State
{
  BOOT,
  IDLE,
  CONNECTING,
  ACTIVE,
  SETTINGSMENU,
  SETTINGADJUST,
  SETTINGAPPLY,
  CALIBRATE
};

State state = State::BOOT;
int configIndex = 0;
bool messageReceived = false;
String message = "";

// Define pins

// Remote HW
#define stickButtonPin 7
#define stickX A0
#define stickY A1
#define batteryPin A2

// Display
#define OLED_RST 19
#define OLED_SDA 20
#define OLED_SCL 21

// LoRa Radio
#define NSS 5
#define RST 17
#define DIO 16

// XMP Images
#define yee_width 64
#define yee_height 64
static unsigned char yee_bits[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xe0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x1f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0x01, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xf0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x18, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x86, 0x3f, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x4e, 0x1f, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x9a, 0x10, 0x18,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x9a, 0x09, 0x2e, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x3a, 0x06, 0x2e, 0x01, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x2f,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x0c, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x31, 0xc0, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x31, 0xc0, 0x81,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x31, 0x80, 0x85, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x80, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x04,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x11, 0x18, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x33, 0x1c, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x03, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0x40,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0xe0, 0x41, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xfc, 0x7f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf4, 0x3f, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xf4, 0x1f, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xf4, 0x07, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xec, 0x01, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x20, 0x07,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xf0, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x38, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x18,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x80,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x38, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0xf0, 0x01, 0x00, 0x00,
    0x00, 0x04, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x3f, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00,
    0x00, 0x04, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0xe0, 0x1f, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x00,
    0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x06, 0x00, 0x00,
    0x00, 0x00, 0x7c, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x01,
    0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x07, 0x00, 0x20,
    0x00, 0x03, 0xc0, 0x03, 0x80, 0x05, 0x00, 0x60, 0x00, 0x06, 0x80, 0x03,
    0x80, 0x04, 0x00, 0x40, 0x00, 0x0c, 0x00, 0x07};

#define yeeqr_width 64
#define yeeqr_height 64
static unsigned char yeeqr_bits[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x9f, 0x99,
    0x07, 0xf8, 0xff, 0x01, 0x80, 0xff, 0x9f, 0x99, 0x07, 0xf8, 0xff, 0x01,
    0x80, 0x01, 0x18, 0xfe, 0x87, 0x19, 0x80, 0x01, 0x80, 0x01, 0x18, 0xfe,
    0x87, 0x19, 0x80, 0x01, 0x80, 0xf9, 0x99, 0x1f, 0x7e, 0x98, 0x9f, 0x01,
    0x80, 0xf9, 0x99, 0x1f, 0x7e, 0x98, 0x9f, 0x01, 0x80, 0xf9, 0x99, 0x9f,
    0x1f, 0x98, 0x9f, 0x01, 0x80, 0xf9, 0x99, 0x9f, 0x1f, 0x98, 0x9f, 0x01,
    0x80, 0xf9, 0x99, 0xe1, 0xe7, 0x99, 0x9f, 0x01, 0x80, 0xf9, 0x99, 0xe1,
    0xe7, 0x99, 0x9f, 0x01, 0x80, 0x01, 0x18, 0x78, 0x06, 0x18, 0x80, 0x01,
    0x80, 0x01, 0x18, 0x78, 0x06, 0x18, 0x80, 0x01, 0x80, 0xff, 0x9f, 0x99,
    0x99, 0xf9, 0xff, 0x01, 0x80, 0xff, 0x9f, 0x99, 0x99, 0xf9, 0xff, 0x01,
    0x00, 0x00, 0x00, 0x98, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x98,
    0x01, 0x00, 0x00, 0x00, 0x80, 0x7f, 0x98, 0xf9, 0xf9, 0x87, 0x9f, 0x01,
    0x80, 0x7f, 0x98, 0xf9, 0xf9, 0x87, 0x9f, 0x01, 0x80, 0x9f, 0x07, 0x98,
    0x81, 0x61, 0x60, 0x00, 0x80, 0x9f, 0x07, 0x98, 0x81, 0x61, 0x60, 0x00,
    0x00, 0x80, 0xf9, 0xff, 0x01, 0xfe, 0x01, 0x00, 0x00, 0x80, 0xf9, 0xff,
    0x01, 0xfe, 0x01, 0x00, 0x00, 0x60, 0x66, 0x1e, 0x7e, 0xfe, 0x1f, 0x00,
    0x00, 0x60, 0x66, 0x1e, 0x7e, 0xfe, 0x1f, 0x00, 0x00, 0x1e, 0xf8, 0x9f,
    0x1f, 0x9e, 0xf9, 0x01, 0x00, 0x1e, 0xf8, 0x9f, 0x1f, 0x9e, 0xf9, 0x01,
    0x00, 0x86, 0x61, 0xe6, 0x79, 0xfe, 0x81, 0x01, 0x00, 0x86, 0x61, 0xe6,
    0x79, 0xfe, 0x81, 0x01, 0x00, 0x9e, 0xf9, 0x7f, 0x9e, 0x01, 0x78, 0x00,
    0x00, 0x9e, 0xf9, 0x7f, 0x9e, 0x01, 0x78, 0x00, 0x80, 0x61, 0x06, 0x7e,
    0xf8, 0xff, 0x87, 0x01, 0x80, 0x61, 0x06, 0x7e, 0xf8, 0xff, 0x87, 0x01,
    0x00, 0xf8, 0xf9, 0x9f, 0xf9, 0xff, 0xff, 0x01, 0x00, 0xf8, 0xf9, 0x9f,
    0xf9, 0xff, 0xff, 0x01, 0x00, 0x00, 0x80, 0x01, 0x80, 0x81, 0x81, 0x01,
    0x00, 0x00, 0x80, 0x01, 0x80, 0x81, 0x81, 0x01, 0x80, 0xff, 0x1f, 0x1e,
    0x9e, 0x99, 0xff, 0x01, 0x80, 0xff, 0x1f, 0x1e, 0x9e, 0x99, 0xff, 0x01,
    0x80, 0x01, 0x18, 0x60, 0xfe, 0x81, 0x01, 0x00, 0x80, 0x01, 0x18, 0x60,
    0xfe, 0x81, 0x01, 0x00, 0x80, 0xf9, 0x19, 0x1e, 0x9e, 0xff, 0x61, 0x00,
    0x80, 0xf9, 0x19, 0x1e, 0x9e, 0xff, 0x61, 0x00, 0x80, 0xf9, 0x99, 0x19,
    0x00, 0x9e, 0xf9, 0x01, 0x80, 0xf9, 0x99, 0x19, 0x00, 0x9e, 0xf9, 0x01,
    0x80, 0xf9, 0x99, 0x07, 0x78, 0xfe, 0x7f, 0x00, 0x80, 0xf9, 0x99, 0x07,
    0x78, 0xfe, 0x7f, 0x00, 0x80, 0x01, 0x98, 0xe1, 0x1f, 0x86, 0x19, 0x00,
    0x80, 0x01, 0x98, 0xe1, 0x1f, 0x86, 0x19, 0x00, 0x80, 0xff, 0x9f, 0x7f,
    0x00, 0xfe, 0xff, 0x01, 0x80, 0xff, 0x9f, 0x7f, 0x00, 0xfe, 0xff, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Function prototype required for compilation
void transitionState(State newState);

void sendHeartbeat()
{
  LoRa.beginPacket();
  LoRa.write(comms.getHeartbeat());
  LoRa.endPacket();
  LoRa.receive();
}

// LoRa received packet IRQ
void onReceive(int packetSize)
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  for (int i = 0; i < packetSize; i++)
  {
    message.concat((char)LoRa.read());
  }
  receivedSize = packetSize;
  messageReceived = true;
}

// Handle received messages
void handleMessage()
{
  if (messageReceived)
  {
    // Single bytes are always heartbeats
    if (receivedSize == 1)
    {
      // Remote always works to match state received in mirror heartbeat
      comms.parseHeartbeat(message[0]);
      heartbeatReceived = true;
      switch (comms.getRemoteHeartbeatMode())
      {
      case CommunicationProtocol::HeartbeatMode::ACTIVE:
        transitionState(State::ACTIVE);
        slowMode = false;
        break;
      case CommunicationProtocol::HeartbeatMode::INACTIVE:
        if (state != State::CONNECTING)
        {
          transitionState(State::IDLE);
        }
        else
        {
          transitionState(State::ACTIVE);
          sendHeartbeat();
        }
        break;
      case CommunicationProtocol::HeartbeatMode::SLOW:
        transitionState(State::ACTIVE);
        slowMode = true;
        break;
      case CommunicationProtocol::HeartbeatMode::CONFIGURE:
        if (comms.getRemoteHeartbeatConfigIndex() == 0b0001)
        {
          transitionState(State::CALIBRATE);
        }
        else
        {
          transitionState(State::SETTINGSMENU);
        }
        break;
      }
    }
    else if (receivedSize == 2 && state == State::ACTIVE)
    {
      xPos = int8_t(message[0]);
      yPos = int8_t(message[1]);
      xPos = map(xPos, -100, 100, 0, 64);
      yPos = map(yPos, -100, 100, 0, 64);
#ifdef DEBUG
      Serial.print("xPos: ");
      Serial.println(xPos);
      Serial.print("yPos: ");
      Serial.println(yPos);
#endif
    }
  }
  message = "";
  messageReceived = false;
}

// Handles all state machine transitions
void transitionState(State newState)
{
  switch (newState)
  {
  case State::ACTIVE:
    if (slowMode)
    {
      comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::SLOW);
    }
    else
    {
      comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::ACTIVE);
    }
    state = State::ACTIVE;
    break;
  case State::IDLE:
    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::INACTIVE);
    state = State::IDLE;
    break;
  case State::CONNECTING:
    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::ACTIVE);
    state = State::CONNECTING;
    break;
  case State::SETTINGSMENU:
    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::CONFIGURE);
    state = State::SETTINGSMENU;
    stick.clearStick();
    sendHeartbeat();
    break;
  case State::SETTINGADJUST:
    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::CONFIGURE);
    state = State::SETTINGADJUST;
    break;
  case State::SETTINGAPPLY:
    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::CONFIGURE);
    state = State::SETTINGAPPLY;
    break;
  case State::CALIBRATE:
    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::CONFIGURE);
    comms.setHeartbeatConfigIndex(0b0001);
    state = State::CALIBRATE;
    sendHeartbeat();
    break;
  }
}

void sendControlPacket()
{
  if ((state == State::ACTIVE || state == State::CALIBRATE) && !zeroSent)
  {
    LoRa.beginPacket();
    LoRa.write(stick.getStickX());
    LoRa.write(stick.getStickY());
    LoRa.endPacket();
    LoRa.receive();
  }
  if (stick.getStickX() == 0 && stick.getStickY() == 0)
  {
    zeroSent = true;
  }
  else
  {
    zeroSent = false;
  }
}

// Used to interface with the button library
void stickClicked()
{
  stick.setStickClicked();
}

void stickHeld()
{
  stick.setStickHeld();
}

// DISPLAY FUNCTIONS

// Prints a battery icon based on voltage
// Likely needs to be much better calibrated
void drawBatteryIcon(float batteryVoltage, int x, int y)
{
  int iconOffset = 0;
  if (batteryVoltage < 3.5)
  {
    iconOffset = 0;
  }
  else if (batteryVoltage < 3.6)
  {
    iconOffset = 1;
  }
  else if (batteryVoltage < 3.65)
  {
    iconOffset = 2;
  }
  else if (batteryVoltage < 3.70)
  {
    iconOffset = 3;
  }
  else if (batteryVoltage < 3.75)
  {
    iconOffset = 4;
  }
  else if (batteryVoltage < 4.2)
  {
    iconOffset = 5;
  }
  else if (batteryVoltage >= 4.2)
  {
    iconOffset = 6;
  }
  u8g2.setFont(u8g2_font_battery19_tn);
  u8g2.drawGlyph(x, y, 0x0030 + iconOffset);
  u8g2.setFont(u8g2_font_helvB12_tr);
}

void drawHUD()
{
  remoteBatteryVoltage = float(remoteBatteryRaw) / 4095 * 3.3 * 2.22;
  drawBatteryIcon(remoteBatteryVoltage, 116, 42);

  if (comms.getRemoteHeartbeatData() != 0)
  {
    mirrorBattery = float(comms.getRemoteHeartbeatData()) / 4095 * 3.3 * 2;
  }
  drawBatteryIcon(mirrorBattery, 116, 64);
  u8g2.setFont(u8g2_font_iconquadpix_m_all);
  u8g2.drawGlyph(100, 64, 0x004a);
  u8g2.drawGlyph(100, 42, 0x0071);
  u8g2.setFont(u8g2_font_streamline_all_t);
  if (heartbeatReceived)
  {
    u8g2.drawGlyph(108, 21, 0x0142);
  }
  else
  {
    u8g2.drawGlyph(108, 21, 0x01e2);
  }
}

void displayBoot()
{
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 + 6, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "LoRa Mirror");
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 + 30, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "mikaeel.io");
}

void displayIdle()
{
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawStr(0, 16, "Click to start");

  drawHUD();

  if (stick.getStickState(false) == Joystick::StickState::CLICKED)
  {
    transitionState(State::CONNECTING);
  }
  if (stick.getStickState() == Joystick::StickState::HELD)
  {
    transitionState(State::SETTINGSMENU);
  }
}

void displayActive()
{
  u8g2.drawFrame(0, 0, 64, 64);
  u8g2.drawHLine(0, 64 - yPos, 64);
  u8g2.drawVLine(xPos, 0, 64);
  u8g2.drawLine(32, 32, xPos, 64 - yPos);
  drawHUD();
  if (stick.getStickState(false) == Joystick::StickState::CLICKED)
  {
    slowMode = !slowMode;
    transitionState(State::ACTIVE);
    sendHeartbeat();
  }
  if (stick.getStickState() == Joystick::StickState::HELD)
  {
    transitionState(State::SETTINGSMENU);
  }
  if (slowMode)
  {
    u8g2.setFont(u8g2_font_helvB12_tr);
    u8g2.drawStr(66, 16, "Slow");
  }
}
void displayConnecting()
{
  u8g2.setFont(u8g2_font_helvB12_tf);
  u8g2.drawStr(0, 16, "Connecting...");
  u8g2.drawStr(0, 38, "Please Wait");
  drawHUD();
  if (stick.getStickState() == Joystick::StickState::HELD)
  {
    transitionState(State::IDLE);
  }
}

void displaySettings()
{
  u8g2.setFont(u8g2_font_twelvedings_t_all);
  u8g2.drawGlyph(1, 11, 0x0047);
  if (menuIndex >= 2)
  {
    u8g2.drawGlyph(u8g2.getDisplayWidth() - 12, 11, 0x007B);
  }
  if (menuIndex <= menuItemsCount - 3)
  {
    u8g2.drawGlyph(u8g2.getDisplayWidth() - 12, u8g2.getDisplayHeight(), 0x007D);
  }
  u8g2.setFont(u8g2_font_helvB12_tr);
  switch (stick.getStickState(false))
  {
  case Joystick::StickState::UP:
    if (menuIndex >= 1)
    {
      menuIndex--;
    }
    stick.clearStick();
    break;
  case Joystick::StickState::DOWN:
    if (menuIndex <= menuItemsCount - 2)
    {
      menuIndex++;
    }
    stick.clearStick();
    break;
  case Joystick::StickState::CLICKED:
    transitionState(State::SETTINGADJUST);
    settingVariable = -1;
    stick.clearStick();
    break;
  case Joystick::StickState::HELD:
    transitionState(State::ACTIVE);
    sendHeartbeat();
    stick.clearStick();
    break;
  }
  if (menuIndex >= 1)
  {
    u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 - 16, U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, configStrings[menuIndex - 1].c_str());
  }
  u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 + 6, U8G2_BTN_INV | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, configStrings[menuIndex].c_str());
  if (menuIndex <= menuItemsCount - 2)
  {
    u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 + 28, U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, configStrings[menuIndex + 1].c_str());
  }
}

void displaySettingAdjust()
{
  switch (menuIndex)
  {
  case 1:
    u8g2.setFont(u8g2_font_helvB12_tr);
    u8g2.drawStr(0, 16, "Batteries");
    remoteBatteryVoltage = float(remoteBatteryRaw) / 4095 * 3.3 * 2.22;
    if (comms.getRemoteHeartbeatData() != 0)
    {
      mirrorBattery = float(comms.getRemoteHeartbeatData()) / 4095 * 3.3 * 2;
    }
    u8g2.setCursor(0, 38);
    u8g2.print("Remote: ");
    u8g2.print(remoteBatteryVoltage);
    u8g2.print("V");
    u8g2.setCursor(0, 60);
    u8g2.print("Mirror: ");
    u8g2.print(mirrorBattery);
    u8g2.print("V");
    if (stick.getStickState() == Joystick::StickState::CLICKED)
    {
      transitionState(State::SETTINGAPPLY);
    }
    break;
  case 2:
    transitionState(State::CALIBRATE);
    break;
  case 3:
    u8g2.drawStr(0, 16, "Centering Stick");
    u8g2.drawStr(0, 38, "Do not touch");
    t.setTimeout([]()
                 { transitionState(State::SETTINGAPPLY); },
                 1000);
    t.setTimeout([]()
                 { stick.centerStick(); },
                 500);
    break;
  case 4:
    if (settingVariable == -1)
    {
      settingVariable = stick.getDeadzone();
    }
    if (stick.getStickState(false) == Joystick::StickState::UP && settingVariable < 64)
    {
      settingVariable++;
      stick.clearStick();
    }
    else if (stick.getStickState(false) == Joystick::StickState::DOWN && settingVariable > 1)
    {
      settingVariable--;
      stick.clearStick();
    }
    else if (stick.getStickState(false) == Joystick::StickState::CLICKED)
    {
      stick.setDeadzone(settingVariable);
      transitionState(State::SETTINGAPPLY);
      stick.clearStick();
    }
    u8g2.drawStr(0, 16, "Deadzone");
    u8g2.drawStr(0, 38, ("Current: " + String(stick.getDeadzone())).c_str());
    u8g2.drawStr(0, 60, ("New: " + String(settingVariable)).c_str());
    break;
  case 5:
    switch (stick.getStickState(false))
    {
    case Joystick::StickState::UP:
      if (aboutIndex >= 1)
      {
        aboutIndex--;
      }
      stick.clearStick();
      break;
    case Joystick::StickState::DOWN:
      if (aboutIndex <= aboutItemsCount - 1)
      {
        aboutIndex++;
      }
      stick.clearStick();
      break;
    case Joystick::StickState::CLICKED:
      transitionState(State::SETTINGSMENU);
      stick.clearStick();
      break;
    case Joystick::StickState::HELD:
      transitionState(State::SETTINGAPPLY);
      sendHeartbeat();
      stick.clearStick();
      break;
    }
    u8g2.setFont(u8g2_font_twelvedings_t_all);
    if (aboutIndex > 0)
    {
      u8g2.drawGlyph(u8g2.getDisplayWidth() - 12, 11, 0x007B);
    }
    if (aboutIndex <= aboutItemsCount - 2)
    {
      u8g2.drawGlyph(u8g2.getDisplayWidth() - 12, u8g2.getDisplayHeight(), 0x007D);
    }
    u8g2.setFont(u8g2_font_helvB12_tr);
    switch (aboutIndex)
    {
    case 0:
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "LoRa Mirror");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 + 18, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "V1.0");
      break;
    case 1:
      u8g2.setFont(u8g2_font_helvB10_tr);
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, 10, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "Made with");
      u8g2.setFont(u8g2_font_streamline_all_t);
      u8g2.drawGlyph(56, 34, 0x017A);
      u8g2.setFont(u8g2_font_helvB10_tr);
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, 46, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "by Mikaeel");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, 62, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "@ The OC Lab");
      break;
    case 2:
      u8g2.setFont(u8g2_font_helvB10_tr);
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, 12, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, "Thanks to");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 4, 26, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth()/2, 0, 1, "Shivun");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 4 * 3, 26, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth()/2, 0, 1, "Alice");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 4, 40, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth()/2, 0, 1, "Kim");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 4 * 3, 40, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth()/2, 0, 1, "Fortune");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 4, 54, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth()/2, 0, 1, "Mitch");
      u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 4 * 3, 54, U8G2_BTN_BW0 | U8G2_BTN_HCENTER, u8g2.getDisplayWidth()/2, 0, 1, "Steve");
      break;
    case 3:
    u8g2.setFont(u8g2_font_helvB12_tr);
      u8g2.drawStr(0, 12, "For all the:");
      u8g2.drawStr(0, 30, "help, lunches,");
      u8g2.drawStr(0, 48, "and fun times!");
      break;
    case 4:
    u8g2.setFont(u8g2_font_helvB12_tr);
      u8g2.drawStr(0, 12, "P.S:");
      u8g2.drawStr(0, 30, "Hold down");
      u8g2.drawStr(0, 48, "the stick");
      break;
    case 5:
    u8g2.setFont(u8g2_font_helvB12_tr);
      u8g2.drawStr(0, 12, "You're still");
      u8g2.drawStr(0, 28, "here?");
      u8g2.drawStr(0, 48, "It's over.");
      u8g2.drawStr(0, 64, "Go home.");
      break;
    }
    break;
  default:
    transitionState(State::SETTINGAPPLY);
    break;
  }
}

void displaySettingApply()
{
  switch (menuIndex)
  {
  case 0:
    u8g2.drawStr(0, 16, "Sleeping");
    t.setTimeout([]()
                 {
                         transitionState(State::IDLE);
                         sendHeartbeat(); },
                 1000);
    break;
  case 2:
    u8g2.drawStr(0, 16, "Saving");
    comms.setHeartbeatConfigIndex(0b1111);
    t.setTimeout([]()
                 { transitionState(State::SETTINGSMENU); },
                 1000);
    break;
  case 3:
    u8g2.drawStr(0, 16, "Centered");
    t.setTimeout([]()
                 { transitionState(State::SETTINGSMENU); },
                 1000);
    break;
  case 4:
    u8g2.drawStr(0, 16, "Deadzone set");
    t.setTimeout([]()
                 { transitionState(State::SETTINGSMENU); },
                 1000);
    break;
  case 5:
    u8g2.drawXBM(0, 0, 64, 64, yee_bits);
    u8g2.drawXBM(64, 0, 64, 64, yeeqr_bits);
    if (stick.getStickState() == Joystick::StickState::CLICKED)
    {
      transitionState(State::SETTINGSMENU);
    }
    break;
  default:
    transitionState(State::SETTINGSMENU);
    break;
  }
}

void displayCalibrate()
{
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.drawStr(0, 16, "Calibrating...");
  if (stick.getStickState() == Joystick::StickState::CLICKED)
  {
    transitionState(State::SETTINGAPPLY);
  }
}

// Updates display based on state machine
void updateDisplay()
{
  u8g2.clearBuffer();
  switch (state)
  {
  case State::BOOT:
    displayBoot();
    break;
  case State::IDLE:
    displayIdle();
    break;
  case State::CONNECTING:
    displayConnecting();
    break;
  case State::ACTIVE:
    displayActive();
    break;
  case State::SETTINGSMENU:
    displaySettings();
    break;
  case State::SETTINGADJUST:
    displaySettingAdjust();
    break;
  case State::SETTINGAPPLY:
    displaySettingApply();
    break;
  case State::CALIBRATE:
    displayCalibrate();
    break;
  }
  u8g2.sendBuffer();
}

void setup()
{
// Serial for debugging
#ifdef DEBUG
  Serial.begin(9600);
#endif

  // Setup LoRa
  pinMode(LED_BUILTIN, OUTPUT);
  SPI.setRX(4);
  SPI.setTX(3);
  SPI.setSCK(2);
  SPI.setCS(5);
  LoRa.setPins(NSS, RST, DIO);
  if (!LoRa.begin(833E6))
  {
#ifdef DEBUG
    Serial.println("Starting LoRa failed!");
#endif
    while (1)
      ;
  }
  // These should be tweaked as a tradeoff between data rate and range
  //  LoRa.setSpreadingFactor(12);
  //  LoRa.setSignalBandwidth(20.8E3);
  LoRa.setTxPower(20);
  LoRa.onReceive(onReceive);
  LoRa.receive();

  // Setup Stick Interrupts
  button.attach(stickButtonPin, INPUT_PULLUP);
  button.callback(stickClicked, SINGLE_TAP);
  button.callback(stickHeld, HOLD);
  button.interval(5);
  stick.centerStick();

  // Initialize Async timers
  t.setInterval(sendControlPacket, 200);
  t.setInterval(updateDisplay, 50);
  heartbeatReceiveTimeout = t.setInterval([]()
                                          { heartbeatReceived = false; },
                                          10000);
  t.setTimeout([]()
               { transitionState(State::IDLE); },
               2000);
  t.setInterval([]()
                { remoteBatteryRaw = analogRead(batteryPin); },
                1500);

  // Setup Display
  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvB12_tr);
}

void loop()
{
  handleMessage();
  stick.updateStick();
  t.handle();
  button.update();
}
