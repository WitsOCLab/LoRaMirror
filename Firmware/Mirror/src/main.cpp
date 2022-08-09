#define DEBUG

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <AsyncTimer.h>
#include <AccelStepper.h>
#include <pico/stdlib.h>
#include <EEPROM.h>

#include "CommunicationProtocol.h"

CommunicationProtocol comms;
AsyncTimer t;
AccelStepper stepper(AccelStepper::FULL4WIRE, 18, 20, 19, 21);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 7, 11, 9, 13);

#define NSS 5
#define RST 17
#define DIO 16
#define LED_PIN 14
#define BATTERY_PIN A0

struct Endstops
{
    int xMin;
    int xMax;
    int yMin;
    int yMax;
};

struct Position
{
    int x;
    int y;
};

enum class State
{
    SLEEP,
    IDLE,
    ACTIVE,
    SLOW,
    CONFIGURE,
    CALIBRATE
};

Endstops endstops = {-4096, 4096, -4096, 4096};
Position position = {0, 0};

State state = State::SLEEP;

int8_t received[2];
int x = 0;
int y = 0;
bool slowMode = false;
word batteryReading = 0;

int sleepTimer = 0;
int timeoutTimer = 0;
int heartbeatTimer = 0;
int positionTimer = 0;

int receivedPacketSize = 0;
bool messageReceived = false;

int positionLocation = 0;
int endstopLocation = positionLocation + sizeof(position);

// #define ModeMask 0b11 << 6
// #define HeartbeatIndexMask 0b11 << 4
// #define DataNibbleMask 0b1111

void onReceive(int packetSize)
{
    for (int i = 0; i < packetSize; i++)
    {
        received[i] = LoRa.read();
    }
    receivedPacketSize = packetSize;
    messageReceived = true;
}

void sleepRadio()
{
    if (state == State::SLEEP)
    {
#ifdef DEBUG
        Serial.println("Sleeping Radio");
#endif
        LoRa.sleep();
    }
}

void retreiveEEPROMData()
{
    EEPROM.get(positionLocation, position);
    EEPROM.get(endstopLocation, endstops);
#ifdef DEBUG
    Serial.print("Retreived EEPROM data: ");
    Serial.print("Position: ");
    Serial.print(position.x);
    Serial.print(", ");
    Serial.println(position.y);
    Serial.print("Endstops: ");
    Serial.print(endstops.xMin);
    Serial.print(" ");
    Serial.print(endstops.xMax);
    Serial.print(", ");
    Serial.print(endstops.yMin);
    Serial.print(" ");
    Serial.println(endstops.yMax);
#endif
}

void sendPosition()
{
    int xp = stepper.currentPosition();
    int yp = -stepper2.currentPosition();
#ifdef DEBUG
    Serial.print("Position: ");
    Serial.print(xp);
    Serial.print(", ");
    Serial.println(yp);
    Serial.print("Endstops: ");
    Serial.print(endstops.xMin);
    Serial.print(", ");
    Serial.print(endstops.xMax);
    Serial.print(", ");
    Serial.print(endstops.yMin);
    Serial.print(", ");
    Serial.println(endstops.yMax);
#endif
    int8_t xpos = int8_t(map(xp, endstops.xMin, endstops.xMax, -100, 100));
    int8_t ypos = int8_t(map(yp, endstops.yMin, endstops.yMax, -100, 100));

#ifdef DEBUG
    Serial.print("Sending Position: ");
    Serial.print(xpos);
    Serial.print(", ");
    Serial.println(ypos);
#endif

    LoRa.beginPacket();
    LoRa.write(xpos);
    LoRa.write(ypos);
    LoRa.endPacket();
    LoRa.receive();
}

void handleState()
{
    switch (state)
    {
    case State::IDLE:
        stepper.disableOutputs();
        stepper2.disableOutputs();
        break;
    case State::ACTIVE:
        if (!(stepper.currentPosition() >= endstops.xMax && stepper.speed() > 0) && !(stepper.currentPosition() <= endstops.xMin && stepper.speed() < 0))
        {
            stepper.runSpeed();
        }
        if (!(-stepper2.currentPosition() >= endstops.yMax && stepper2.speed() < 0) && !(-stepper2.currentPosition() <= endstops.yMin && stepper2.speed() > 0))
        {
            stepper2.runSpeed();
        }
        break;
    case State::CALIBRATE:
        stepper.runSpeed();
        stepper2.runSpeed();
    }
}

word getBatteryVoltage()
{
    batteryReading = analogRead(BATTERY_PIN);
#ifdef DEBUG
    Serial.print("Battery: ");
    Serial.println(batteryReading);
#endif
    return batteryReading;
}

void sendHeartbeat()
{
    digitalWrite(LED_PIN, HIGH);

    if (comms.getHeartbeatIndex() == 0)
    {
        comms.setHeartbeatData(getBatteryVoltage());
    }
    LoRa.beginPacket();
    LoRa.write(comms.getHeartbeat());
    LoRa.endPacket();
    LoRa.receive();
    if (state == State::SLEEP)
    {
        delay(150);
        sleepRadio();
        digitalWrite(LED_PIN, LOW);
    }
}

// Function prototype
void transitionState(State(newState));

void sleepTimeout()
{
#ifdef DEBUG
    Serial.println("Sleep timeout");
#endif
    transitionState(State::SLEEP);
}

void controlTimeout()
{
#ifdef DEBUG
    Serial.println("Control timeout");
#endif
    x = 0;
    y = 0;
    transitionState(State::IDLE);
}

void transitionState(State newState)
{
    if (state != newState)
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
            digitalWrite(LED_PIN, HIGH);
            t.cancel(sleepTimer);
            timeoutTimer = t.setTimeout(controlTimeout, 500);
#ifdef DEBUG
            Serial.println("Transitioned to ACTIVE");
#endif
            break;
        case State::IDLE:
            if (slowMode)
            {
                comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::SLOW);
            }
            else
            {
                comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::ACTIVE);
            }
            state = State::IDLE;
            digitalWrite(LED_PIN, HIGH);
            position.x = stepper.currentPosition();
            position.y = -stepper2.currentPosition();
            EEPROM.put(positionLocation, position);
            EEPROM.put(endstopLocation, endstops);
            sleepTimer = t.setTimeout(sleepTimeout, 120000);
            t.cancel(timeoutTimer);
            t.setTimeout([]()
                         { sendPosition(); },
                         250);
#ifdef DEBUG
            Serial.println("Transitioned to IDLE");
#endif
            break;
        case State::SLEEP:
            comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::INACTIVE);
            state = State::SLEEP;
            digitalWrite(LED_PIN, LOW);
            t.cancel(timeoutTimer);
            t.cancel(sleepTimer);
            t.cancel(positionTimer);
            t.reset(heartbeatTimer);
            EEPROM.put(positionLocation, position);
            EEPROM.put(endstopLocation, endstops);
            EEPROM.commit();
            sendHeartbeat();
#ifdef DEBUG
            Serial.println("Transitioned to SLEEP");
#endif
            break;
        case State::CONFIGURE:
            comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::CONFIGURE);
            comms.setHeartbeatConfigIndex(0);
            state = State::CONFIGURE;
            digitalWrite(LED_PIN, LOW);
            t.cancel(timeoutTimer);
            t.cancel(positionTimer);
            t.reset(heartbeatTimer);
            t.reset(sleepTimer);
            sendHeartbeat();
#ifdef DEBUG
            Serial.println("Transitioned to CONFIGURE");
#endif
            break;
        case State::CALIBRATE:
            comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::CONFIGURE);
            comms.setHeartbeatConfigIndex(0b0001);
            state = State::CALIBRATE;
            digitalWrite(LED_PIN, HIGH);
            t.cancel(timeoutTimer);
            t.cancel(positionTimer);
            t.reset(heartbeatTimer);
            t.reset(sleepTimer);

            endstops.xMax = 0;
            endstops.yMax = 0;
            endstops.xMin = 0;
            endstops.yMin = 0;

            stepper.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);

            sendHeartbeat();
#ifdef DEBUG
            Serial.println("Transitioned to CALIBRATE");
#endif
            break;
        }
    }
}

void handleMessage()
{
    if (messageReceived)
    {
        messageReceived = false;
        switch (state)
        {
        case State::ACTIVE:
        case State::IDLE:
            t.reset(timeoutTimer);
            if (receivedPacketSize == 1)
            {
                comms.parseHeartbeat(received[0]);
                switch (comms.getRemoteHeartbeatMode())
                {
                case CommunicationProtocol::HeartbeatMode::INACTIVE:
#ifdef DEBUG
                    Serial.println("Received Deactivation");
#endif
                    sleepTimeout();
                    break;
                case CommunicationProtocol::HeartbeatMode::SLOW:
                    slowMode = true;
                    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::SLOW);
                    sendHeartbeat();
#ifdef DEBUG
                    Serial.println("Received Slow Mode");
#endif
                    break;
                case CommunicationProtocol::HeartbeatMode::ACTIVE:
                    slowMode = false;
                    comms.setHeartbeatMode(CommunicationProtocol::HeartbeatMode::ACTIVE);
                    sendHeartbeat();
#ifdef DEBUG
                    Serial.println("Received Active Mode");
#endif
                    break;
                case CommunicationProtocol::HeartbeatMode::CONFIGURE:
                    transitionState(State::CONFIGURE);
                    sendHeartbeat();
#ifdef DEBUG
                    Serial.println("Received Configure Mode");
#endif
                    break;
                }
            }
            else if (receivedPacketSize == 2)
            {
                x = received[0];
                y = received[1];
                if (slowMode)
                {
                    stepper.setSpeed(0.1 * x);
                    stepper2.setSpeed(-0.1 * y);
                }
                else
                {
                    stepper.setSpeed(4 * x);
                    stepper2.setSpeed(-4 * y);
                }

                if (x == 0 && y == 0)
                {
                    transitionState(State::IDLE);
                }
                else
                {
                    t.reset(sleepTimer);
                    transitionState(State::ACTIVE);
                }
            }
            break;
        case State::SLEEP:
            comms.parseHeartbeat(received[0]);
            if ((comms.getRemoteHeartbeatMode() == CommunicationProtocol::HeartbeatMode::ACTIVE) || (comms.getRemoteHeartbeatMode() == CommunicationProtocol::HeartbeatMode::SLOW))
            {
                transitionState(State::IDLE);
                LoRa.receive();
            }
            break;
        case State::CONFIGURE:
            comms.parseHeartbeat(received[0]);
            switch (comms.getRemoteHeartbeatMode())
            {
            case CommunicationProtocol::HeartbeatMode::INACTIVE:
#ifdef DEBUG
                Serial.println("Received Deactivation");
#endif
                sleepTimeout();
                break;
            case CommunicationProtocol::HeartbeatMode::CONFIGURE:
                if (comms.getRemoteHeartbeatConfigIndex() == 0b0001)
                {
#ifdef DEBUG
                    Serial.println("Received calibrate commands");
#endif
                    transitionState(State::CALIBRATE);
                }
                break;
            case CommunicationProtocol::HeartbeatMode::ACTIVE:
            case CommunicationProtocol::HeartbeatMode::SLOW:
                transitionState(State::ACTIVE);
            }
            break;
        case State::CALIBRATE:
            if (receivedPacketSize == 1)
            {
                comms.parseHeartbeat(received[0]);
                if (comms.getRemoteHeartbeatConfigIndex() == 0b1111)
                {
#ifdef DEBUG
                    Serial.println("Received calibration complete");
#endif
                    transitionState(State::IDLE);
                }
                else if (comms.getRemoteHeartbeatMode() == CommunicationProtocol::HeartbeatMode::ACTIVE || comms.getRemoteHeartbeatMode() == CommunicationProtocol::HeartbeatMode::SLOW)
                {
                    transitionState(State::ACTIVE);
                }
            }
            else
            {
                x = received[0];
                y = received[1];
                stepper.setSpeed(4 * x);
                stepper2.setSpeed(-4 * y);
                if (x == 0 and y == 0)
                {
                    if (stepper.currentPosition() < endstops.xMin)
                    {
                        endstops.xMin = stepper.currentPosition();
                    }
                    else if (stepper.currentPosition() > endstops.xMax)
                    {
                        endstops.xMax = stepper.currentPosition();
                    }
                    if (-stepper2.currentPosition() < endstops.yMin)
                    {
                        endstops.yMin = -stepper2.currentPosition();
                    }
                    else if (-stepper2.currentPosition() > endstops.yMax)
                    {
                        endstops.yMax = -stepper2.currentPosition();
                    }

#ifdef DEBUG
                    Serial.print("Position: ");
                    Serial.print(stepper.currentPosition());
                    Serial.print(" ");
                    Serial.println(-stepper2.currentPosition());
                    Serial.print("New Endstops - X: ");
                    Serial.print(endstops.xMin);
                    Serial.print(" ");
                    Serial.print(endstops.xMax);
                    Serial.print(" Y: ");
                    Serial.print(endstops.yMin);
                    Serial.print(" ");
                    Serial.println(endstops.yMax);
#endif
                }
                else
                {
                    t.reset(sleepTimer);
                }
            }
        }
    }
}

void setup()
{
#ifdef DEBUG
    delay(10000);
    Serial.begin(9600);
    Serial.println("LoRa Receiver");
#endif

    pinMode(LED_PIN, OUTPUT);

    // Configure LoRa
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
    LoRa.setTxPower(20);
    LoRa.onReceive(onReceive);
    LoRa.receive();

    EEPROM.begin(512);
    retreiveEEPROMData();

    // Configure Steppers
    stepper.setMaxSpeed(500);
    stepper2.setMaxSpeed(500);
    stepper.setCurrentPosition(position.x);
    stepper2.setCurrentPosition(-position.y);

    // Configure Async timers
    heartbeatTimer = t.setInterval(sendHeartbeat, 5000);
}

void loop()
{
    handleMessage();
    handleState();
    t.handle();
}
