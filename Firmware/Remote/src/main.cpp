#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <AsyncTimer.h>
#include <Yabl.h>

AsyncTimer t;

// some variables
int x = 0;
int y = 0;
int x_center = 0;
int y_center = 0;
int deadzone = 20;
int8_t xSend;
int8_t ySend;
byte statusFlags = 0;
bool zeroSent = false;
String configStrings[6]{"Sleep", "Center Stick", "Calibrate", "Set Deadzone", "Option 5", "Option 6"};
int menuItems = 6;
int menuCounter = 0;
int settingVariable = 0;
bool slowMode = false;
int receivedSize = 0;
int xPos = 0;
int yPos = 0;

enum class HeartbeatMode
{
  INACTIVE,
  ACTIVE,
  SLOW,
  CONFIGURE
};

enum class State
{
  IDLE,
  CONNECTING,
  ACTIVE,
  SETTINGSMENU,
  SETTINGADJUST,
  SETTINGAPPLY,
  CALIBRATE
};

enum class StickState
{
  UP,
  DOWN,
  CLICKED,
  CENTERED,
  CLEARED,
};

State state = State::IDLE;
StickState stickState = StickState::CENTERED;
HeartbeatMode heartbeatMode = HeartbeatMode::INACTIVE;
int heartbeatIndex = 0;
byte heartbeatNibble[4] = {0, 0, 0, 0};
int configIndex = 0;

// define pins
#define stickButtonPin 28
#define stickX A0
#define stickY A1

#define OLED_RST 19
#define OLED_SDA 20
#define OLED_SCL 21

#define NSS 5
#define RST 17
#define DIO 16

#define ActiveFlag 1 << 7
#define SlowModeFlag 1 << 6
#define ConfigureFlag 1 << 5
#define ConfirmFlag 1 << 4

#define InactiveMode 0b00 << 6
#define ActiveMode 0b01 << 6
#define SlowMode 0b10 << 6
#define ConfigureMode 0b11 << 6
#define ModeMask 0b11 << 6
#define HeartbeatIndexMask 0b11 << 4
#define DataNibbleMask 0b1111

bool messageReceived = false;
String message = "";

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
Button button;

void transitionState(State newState);

void parseHeartbeat(byte heartbeat)
{
  heartbeatMode = (HeartbeatMode)((heartbeat & ModeMask) >> 6);
  heartbeatIndex = (heartbeat & HeartbeatIndexMask) >> 4;
  if (heartbeatMode != HeartbeatMode::CONFIGURE)
  {
    heartbeatNibble[heartbeatIndex] = heartbeat & DataNibbleMask;
  }
  else
  {
    configIndex = heartbeat & DataNibbleMask;
  }
  Serial.print("Mode: ");
  Serial.print(int(heartbeatMode), BIN);
  Serial.print(" Index: ");
  Serial.print(heartbeatIndex);
  if (heartbeatMode != HeartbeatMode::CONFIGURE)
  {
    Serial.print(" Data: ");
    Serial.println(heartbeatNibble[heartbeatIndex], BIN);
  }
  else
  {
    Serial.print(" Config Index: ");
    Serial.println(configIndex);
  }
}

void sendHeartbeat(byte data = 0)
{
  statusFlags &= ~DataNibbleMask;
  statusFlags |= data;
  LoRa.beginPacket();
  LoRa.write(statusFlags);
  LoRa.endPacket();
  LoRa.receive();
}

void onReceive(int packetSize)
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // read packet
  for (int i = 0; i < packetSize; i++)
  {
    message.concat((char)LoRa.read());
  }
  receivedSize = packetSize;
  messageReceived = true;
}

void handleMessage()
{
  if (messageReceived)
  {
    // Serial.print("Message received: ");
    // Serial.println(message[0], BIN);
    if (receivedSize == 1)
    {
      parseHeartbeat(message[0]);
    }
    switch (state)
    {
    case State::CONNECTING:
      transitionState(State::ACTIVE);
      sendHeartbeat();
      break;
    case State::ACTIVE:
      if (receivedSize == 1)
      {
        switch (heartbeatMode)
        {
        case HeartbeatMode::INACTIVE:
          transitionState(State::IDLE);
          break;
        case HeartbeatMode::ACTIVE:
          slowMode = false;
          break;
        case HeartbeatMode::SLOW:
          slowMode = true;
          break;
        }
      }
      else if (receivedSize == 2)
      {
        xPos = int8_t(message[0]);
        yPos = int8_t(message[1]);
        xPos = map(xPos, -100, 100, 0, 64);
        Serial.print("xPos: ");
        Serial.println(xPos);
        yPos = map(yPos, -100, 100, 0, 64);
        Serial.print("yPos: ");
        Serial.println(yPos);
      }
      break;
    }
    message = "";
    messageReceived = false;
  }
}

void transitionState(State newState)
{
  switch (newState)
  {
  case State::ACTIVE:
    statusFlags &= ~ModeMask;
    if (slowMode)
    {
      statusFlags |= SlowMode;
    }
    else
    {
      statusFlags |= ActiveMode;
    }
    state = State::ACTIVE;
    break;
  case State::IDLE:
    statusFlags &= ~ModeMask;
    statusFlags |= InactiveMode;
    state = State::IDLE;
    break;
  case State::CONNECTING:
    statusFlags &= ~ModeMask;
    statusFlags |= ActiveMode;
    state = State::CONNECTING;
    break;
  case State::SETTINGSMENU:
    statusFlags &= ~ModeMask;
    statusFlags |= ConfigureMode;
    state = State::SETTINGSMENU;
    sendHeartbeat();
    break;
  case State::SETTINGADJUST:
    statusFlags &= ~ModeMask;
    statusFlags |= ConfigureMode;
    state = State::SETTINGADJUST;
    break;
  case State::SETTINGAPPLY:
    statusFlags &= ~ModeMask;
    statusFlags |= ConfigureMode;
    state = State::SETTINGAPPLY;
    break;
  case State::CALIBRATE:
    statusFlags &= ~ModeMask;
    statusFlags |= ConfigureMode;
    state = State::CALIBRATE;
    sendHeartbeat(0b0001);
    break;
  }
}



void sendControlPacket()
{
  if ((state == State::ACTIVE || state == State::CALIBRATE) && !zeroSent)
  {
    LoRa.beginPacket();
    LoRa.write(xSend);
    LoRa.write(ySend);
    LoRa.endPacket();
    LoRa.receive();
    // Serial.print("Sent: ");
    // Serial.print(xSend);
    // Serial.print(", ");
    // Serial.println(ySend);
  }
  if (xSend == 0 && ySend == 0)
  {
    zeroSent = true;
  }
  else
  {
    zeroSent = false;
  }
}

void centerStick()
{
  delay(100);
  x_center = analogRead(stickX);
  x_center = map(x_center, 0, 4095, 0, 255);
  y_center = analogRead(stickY);
  y_center = map(y_center, 0, 4095, 0, 255);
}

void getStick()
{
  x = analogRead(stickX);
  x = map(x, 0, 4095, 0, 255);
  y = analogRead(stickY);
  y = map(y, 0, 4095, 0, 255);
  x -= x_center;
  y -= y_center;

  // Add deadzone
  if (abs(x) < deadzone)
  {
    x = 0;
  }
  if (abs(y) < deadzone)
  {
    y = 0;
  }

  if (stickState != StickState::CLICKED)
  {
    if (stickState != StickState::CLEARED)
    {
      if (y > 0)
      {
        stickState = StickState::UP;
      }
      else if (y < 0)
      {
        stickState = StickState::DOWN;
      }
      else
      {
        stickState = StickState::CENTERED;
      }
    }
    else
    {
      if (y == 0)
      {
        stickState = StickState::CENTERED;
      }
    }
  }

  xSend = int8_t(map(x, 0, 255, 0, 200));
  ySend = int8_t(map(y, 0, 255, 0, 200));
}

void stickClicked()
{
  switch (state)
  {
  case State::IDLE:
    transitionState(State::CONNECTING);
    break;
  case State::ACTIVE:
    slowMode = !slowMode;
    transitionState(State::ACTIVE);
    sendHeartbeat();
    break;
  case State::SETTINGSMENU:
  case State::SETTINGADJUST:
  case State::CALIBRATE:
    stickState = StickState::CLICKED;
    break;
  }
}

void stickHeld()
{
  switch (state)
  {
  case State::ACTIVE:
    transitionState(State::SETTINGSMENU);
    break;
  case State::CONNECTING:
    transitionState(State::IDLE);
    sendHeartbeat();
    break;
  case State::SETTINGSMENU:
    transitionState(State::ACTIVE);
    sendHeartbeat();
  }
}

void displayIdle()
{
  u8g2.drawStr(0, 20, "Idle");
}

void displayActive()
{
  u8g2.drawFrame(0, 0, 64, 64);
  u8g2.drawHLine(0, 64 - yPos, 64);
  u8g2.drawVLine(xPos, 0, 64);
  u8g2.drawLine(32, 32, xPos, 64 - yPos);
  if (slowMode)
  {
    u8g2.drawStr(70, 20, "Slow");
  }
}
void displayConnecting()
{
  u8g2.drawStr(0, 20, "Connecting...");
}

void displaySettings()
{
  u8g2.setFont(u8g2_font_twelvedings_t_all);
  u8g2.drawGlyph(1, 11, 0x0047);
  if (menuCounter >= 2)
  {
    u8g2.drawGlyph(u8g2.getDisplayWidth() - 12, 11, 0x007B);
  }
  if (menuCounter <= menuItems - 3)
  {
    u8g2.drawGlyph(u8g2.getDisplayWidth() - 12, u8g2.getDisplayHeight(), 0x007D);
  }
  u8g2.setFont(u8g2_font_helvB12_tr);
  if (stickState == StickState::UP && menuCounter >= 1)
  {
    menuCounter--;
    stickState = StickState::CLEARED;
  }
  else if (stickState == StickState::DOWN && menuCounter <= menuItems - 2)
  {
    menuCounter++;
    stickState = StickState::CLEARED;
  }
  else if (stickState == StickState::CLICKED)
  {
    transitionState(State::SETTINGADJUST);
    settingVariable = -1;
    stickState = StickState::CLEARED;
  }
  if (menuCounter >= 1)
  {
    u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 - 16, U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, configStrings[menuCounter - 1].c_str());
  }
  u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 + 6, U8G2_BTN_INV | U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, configStrings[menuCounter].c_str());
  if (menuCounter <= menuItems - 2)
  {
    u8g2.drawButtonUTF8(u8g2.getDisplayWidth() / 2, u8g2.getDisplayHeight() / 2 + 28, U8G2_BTN_HCENTER, u8g2.getDisplayWidth(), 0, 1, configStrings[menuCounter + 1].c_str());
  }
}

void displaySettingAdjust()
{
  switch (menuCounter)
  {
  case 1:
    u8g2.drawStr(0, 20, "Centering Stick");
    u8g2.drawStr(0, 40, "Do not touch");
    t.setTimeout([]()
                 { transitionState(State::SETTINGAPPLY); },
                 1000);
    t.setTimeout([]()
                 { centerStick(); },
                 500);
    break;
  case 2:
    transitionState(State::CALIBRATE);
  case 3:
    if (settingVariable == -1)
    {
      settingVariable = deadzone;
    }
    if (stickState == StickState::UP && settingVariable < 64)
    {
      settingVariable++;
      stickState = StickState::CLEARED;
    }
    else if (stickState == StickState::DOWN && settingVariable > 1)
    {
      settingVariable--;
      stickState = StickState::CLEARED;
    }
    else if (stickState == StickState::CLICKED)
    {
      deadzone = settingVariable;
      transitionState(State::SETTINGAPPLY);
      stickState = StickState::CLEARED;
    }
    u8g2.drawStr(0, 20, "Deadzone");
    u8g2.drawStr(0, 40, ("Current: " + String(deadzone)).c_str());
    u8g2.drawStr(0, 60, ("New: " + String(settingVariable)).c_str());
    break;
  default:
    transitionState(State::SETTINGAPPLY);
    break;
  }
}

void displaySettingApply()
{
  switch (menuCounter)
  {
  case 0:
    u8g2.drawStr(0, 20, "Sleeping");
    t.setTimeout([]()
                 {
                         transitionState(State::IDLE);
                         sendHeartbeat(); },
                 1000);
    break;
  case 1:
    u8g2.drawStr(0, 20, "Centered");
    t.setTimeout([]()
                 { transitionState(State::SETTINGSMENU); },
                 1000);
    break;
  case 2:
    u8g2.drawStr(0, 20, "Saving");
    sendHeartbeat(0b1111);
    t.setTimeout([]()
                 { transitionState(State::SETTINGSMENU); },
                 1000);
  case 3:
    u8g2.drawStr(0, 20, "Deadzone set");
    t.setTimeout([]()
                 { transitionState(State::SETTINGSMENU); },
                 1000);
    break;
  default:
    transitionState(State::SETTINGSMENU);
    break;
  }
}

void displayCalibrate()
{
  u8g2.drawStr(0, 20, "Calibrating...");
  if (stickState == StickState::CLICKED)
  {
    transitionState(State::SETTINGAPPLY);
    stickState = StickState::CLEARED;
  }
}

void updateDisplay()
{
  u8g2.clearBuffer();
  switch (state)
  {
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
  Serial.begin(9600);

  // Setup LoRa
  pinMode(LED_BUILTIN, OUTPUT);
  SPI.setRX(4);
  SPI.setTX(3);
  SPI.setSCK(2);
  SPI.setCS(5);
  LoRa.setPins(NSS, RST, DIO);
  if (!LoRa.begin(833E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  // LoRa.setSpreadingFactor(12);
  // LoRa.setSignalBandwidth(20.8E3);
  LoRa.setTxPower(20);
  // Register the receive callback
  LoRa.onReceive(onReceive);
  // Put the radio into receive mode
  LoRa.receive();

  // Setup Stick Interrupts
  //  attachInterrupt(digitalPinToInterrupt(stickButtonPin), stickClicked, FALLING);
  //  pinMode(stickButtonPin, INPUT_PULLUP);
  button.attach(stickButtonPin, INPUT_PULLUP);
  button.callback(stickClicked, SINGLE_TAP);
  button.callback(stickHeld, HOLD);
  button.interval(5);

  // Initialize Async timers
  t.setInterval(sendControlPacket, 200);
  t.setInterval(updateDisplay, 50);

  // Setup Display
  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvB12_tr);
  centerStick();
}

void loop()
{
  handleMessage();
  getStick();
  t.handle();
  button.update();
}
