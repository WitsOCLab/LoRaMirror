
/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRa *************************************************************

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x23, 0x92, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x78, 0x56, 0x34, 0x12, 0x89, 0x67, 0x45, 0x23};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x76, 0x4C, 0x6D, 0x8B, 0x2C, 0x3B, 0x71, 0xE7, 0x95, 0x97, 0xD5, 0x42, 0xBF, 0x14, 0xB8, 0x68};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

uint8_t _payload[32]; //max 32 bytes payload
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

// Application ******************************************************

bool _printDebug = true;
// The time while we should output to the serial port, screen, etc.
// After this time, those things are disabled to save power.
long _debugTime = 0;

// ****************************************************************************
// Custom Functions

/// Populate the _payload array with stuff to send and returns how many bytes
/// are to be sent. Keeps track of the state machine of what to send.
u1_t PopulatePayload()
{

  return 0;
}

// ****************************************************************************
// Worker Functions

void PrintlnDebug(String text)
{
  if (_printDebug)
  {
    Serial.println(text);
  }
}

void PrintDebug(String text)
{
  if (_printDebug)
  {
    Serial.print(text);
  }
}

void printHex2(unsigned v)
{
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    PrintlnDebug(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    u1_t bytes = PopulatePayload();
    if (bytes != 0)
    {
      LMIC_setTxData2(1, _payload, bytes, 0);
      PrintlnDebug(F("Packet queued"));
    }
    else
    {
      // Send a blank in case we receive something...
      LMIC_setTxData2(1, 0, 1, 0);
      PrintlnDebug(F("Sending heartbeat"));
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    PrintlnDebug(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    PrintlnDebug(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    PrintlnDebug(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    PrintlnDebug(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    PrintlnDebug(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
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
      for (size_t i = 0; i < sizeof(artKey); ++i)
      {
        if (i != 0)
          Serial.print("-");
        printHex2(artKey[i]);
      }
      Serial.println("");
      Serial.print("NwkSKey: ");
      for (size_t i = 0; i < sizeof(nwkKey); ++i)
      {
        if (i != 0)
          Serial.print("-");
        printHex2(nwkKey[i]);
      }
      Serial.println();
    }
    // Disable link check validation (automatically enabled
    // during join) not supported by TTN
    LMIC_setLinkCheckMode(0);
    break;
  /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
  case EV_JOIN_FAILED:
    PrintlnDebug(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    PrintlnDebug(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    PrintlnDebug(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      PrintlnDebug(F("Received ack"));
    if (LMIC.dataLen)
    {
      PrintDebug(F("Received "));
      PrintDebug(LMIC.dataLen);
      PrintlnDebug(F(" bytes of payload"));
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    PrintlnDebug(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    PrintlnDebug(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    PrintlnDebug(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    PrintlnDebug(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    PrintlnDebug(F("EV_LINK_ALIVE"));
    break;
  /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
  case EV_TXSTART:
    PrintlnDebug(F("EV_TXSTART"));
    break;
  case EV_TXCANCELED:
    PrintlnDebug(F("EV_TXCANCELED"));
    break;
  case EV_RXSTART:
    /* do not print anything -- it wrecks timing */
    break;
  case EV_JOIN_TXCOMPLETE:
    PrintlnDebug(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    break;

  default:
    PrintDebug(F("Unknown event: "));
    PrintlnDebug(String((unsigned)ev));
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting..."));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();



  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop()
{
  os_runloop_once();
  // Don't do more here to keep tight timing.
}
