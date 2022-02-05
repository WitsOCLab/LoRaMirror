#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <AsyncTimer.h>
#include <AccelStepper.h>

AsyncTimer t(100);

#define NSS 5
#define RST 17
#define DIO 16

enum class State
{
    SLEEP,
    IDLE,
    ACTIVE,
    SLOW,
    CONFIGURE,
    CALIBRATE
};

enum class HeartbeatMode
{
    INACTIVE,
    ACTIVE,
    SLOW,
    CONFIGURE,
    CALIBRATE
};

HeartbeatMode heartbeatMode = HeartbeatMode::INACTIVE;

State state = State::SLEEP;

int8_t received[2];
int x = 0;
int y = 0;
bool slowMode = false;

int sleepTimer = 0;
int timeoutTimer = 0;
int heartbeatTimer = 0;
int positionTimer = 0;

byte statusFlags = 0;
byte receivedStatusFlags = 0;
int receivedPacketSize = 0;
bool messageReceived = false;
int heartbeatIndex = 0;
byte heartbeatNibble[4] = {0, 0, 0, 0};
int configIndex = 0;

int xMin = -4096;
int xMax = 4096;
int yMin = -4096;
int yMax = 4096;

#define InactiveMode 0b00 << 6
#define ActiveMode 0b01 << 6
#define SlowMode 0b10 << 6
#define ConfigureMode 0b11 << 6
#define ModeMask 0b11 << 6
#define HeartbeatIndexMask 0b11 << 4
#define DataNibbleMask 0b1111

AccelStepper stepper(AccelStepper::FULL4WIRE, 18, 20, 19, 21); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::FULL4WIRE, 7, 11, 9, 13);  // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void onReceive(int packetSize)
{
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // read packet
    for (int i = 0; i < packetSize; i++)
    {
        received[i] = LoRa.read();
    }
    receivedPacketSize = packetSize;
    messageReceived = true;
}

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

void sleepRadio()
{
    if (state == State::SLEEP)
    {
        Serial.println("Sleeping Radio");
        LoRa.sleep();
    }
}



void sendPosition()
{
    int xp = stepper.currentPosition();
    int yp = stepper2.currentPosition();
    int8_t xpos = int8_t(map(xp, xMin, xMax, -100, 100));
    int8_t ypos = int8_t(map(yp, yMin, yMax, -100, 100));
    Serial.print("Sending Position: ");
    Serial.print(xpos);
    Serial.print(", ");
    Serial.println(ypos);
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
        if (!(stepper.currentPosition() >= xMax && stepper.speed() > 0) && !(stepper.currentPosition() <= xMin && stepper.speed() < 0))
        {
            stepper.runSpeed();
        }
        if (!(stepper2.currentPosition() >= yMax && stepper2.speed() > 0) && !(stepper2.currentPosition() <= yMin && stepper2.speed() < 0))
        {
            stepper2.runSpeed();
        }
        break;
    case State::CALIBRATE:
        stepper.runSpeed();
        stepper2.runSpeed();
    }
}

void sendHeartbeat()
{
    statusFlags &= ~HeartbeatIndexMask;
    statusFlags |= (heartbeatIndex << 4);
    heartbeatIndex++;
    if (heartbeatIndex > 3)
    {
        heartbeatIndex = 0;
    }
    LoRa.beginPacket();
    LoRa.write(statusFlags);
    LoRa.endPacket();
    LoRa.receive();
    Serial.print("Heartbeat sent ");
    Serial.println(statusFlags, BIN);
    if (state == State::SLEEP)
    {
        delay(150);
        sleepRadio();
    }
}

void transitionState(State(newState));

void sleepTimeout()
{
    Serial.println("Sleep timeout");
    transitionState(State::SLEEP);
}

void controlTimeout()
{
    Serial.println("Control timeout");
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
            digitalWrite(LED_BUILTIN, HIGH);
            t.cancel(sleepTimer);
            timeoutTimer = t.setTimeout(controlTimeout, 500);
            Serial.println("Transitioned to ACTIVE");
            break;
        case State::IDLE:
            statusFlags &= ~ModeMask;
            if (slowMode)
            {
                statusFlags |= SlowMode;
            }
            else
            {
                statusFlags |= ActiveMode;
            }
            state = State::IDLE;
            digitalWrite(LED_BUILTIN, HIGH);
            sleepTimer = t.setTimeout(sleepTimeout, 30000);
            t.cancel(timeoutTimer);
            t.setTimeout([]()
                         { sendPosition(); },
                         250);
            Serial.println("Transitioned to IDLE");
            break;
        case State::SLEEP:
            statusFlags &= ~ModeMask;
            statusFlags |= InactiveMode;
            state = State::SLEEP;
            digitalWrite(LED_BUILTIN, LOW);
            t.cancel(timeoutTimer);
            t.cancel(sleepTimer);
            t.cancel(positionTimer);
            t.reset(heartbeatTimer);
            sendHeartbeat();
            Serial.println("Transitioned to SLEEP");
            break;
        case State::CONFIGURE:
            statusFlags &= ~ModeMask;
            statusFlags |= ConfigureMode;
            state = State::CONFIGURE;
            digitalWrite(LED_BUILTIN, LOW);
            t.cancel(timeoutTimer);
            t.cancel(sleepTimer);
            t.cancel(positionTimer);
            t.reset(heartbeatTimer);
            sendHeartbeat();
            Serial.println("Transitioned to CONFIGURE");
            break;
        case State::CALIBRATE:
            statusFlags &= ~ModeMask;
            statusFlags |= ConfigureMode;
            state = State::CALIBRATE;
            digitalWrite(LED_BUILTIN, HIGH);
            t.cancel(timeoutTimer);
            t.cancel(sleepTimer);
            t.cancel(positionTimer);
            t.reset(heartbeatTimer);
            xMax = 0;
            yMax = 0;
            xMin = 0;
            yMin = 0;
            stepper.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);
            sendHeartbeat();
            Serial.println("Transitioned to CALIBRATE");
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
                // Serial.print("Received status ");
                // Serial.println(byte(received[0]), BIN);
                parseHeartbeat(received[0]);
                switch (heartbeatMode)
                {
                case HeartbeatMode::INACTIVE:
                    Serial.println("Received Deactivation");
                    sleepTimeout();
                    break;
                case HeartbeatMode::SLOW:
                    slowMode = true;
                    Serial.println("Received Slow Mode");
                    statusFlags &= ~ModeMask;
                    statusFlags |= SlowMode;
                    sendHeartbeat();
                    break;
                case HeartbeatMode::ACTIVE:
                    slowMode = false;
                    Serial.println("Received Active Mode");
                    statusFlags &= ~ModeMask;
                    statusFlags |= ActiveMode;
                    sendHeartbeat();
                    break;
                case HeartbeatMode::CONFIGURE:
                    Serial.println("Received Configure Mode");
                    transitionState(State::CONFIGURE);
                    sendHeartbeat();
                    break;
                }
            }
            else
            {
                x = received[0];
                y = received[1];
                if (slowMode)
                {
                    stepper.setSpeed(0.1 * x);
                    stepper2.setSpeed(0.1 * y);
                }
                else
                {
                    stepper.setSpeed(4 * x);
                    stepper2.setSpeed(4 * y);
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
            parseHeartbeat(received[0]);
            if ((heartbeatMode == HeartbeatMode::ACTIVE) || (heartbeatMode == HeartbeatMode::SLOW))
            {
                transitionState(State::IDLE);
                LoRa.receive();
            }
            break;
        case State::CONFIGURE:
            parseHeartbeat(received[0]);
            switch (heartbeatMode)
            {
            case HeartbeatMode::INACTIVE:
                Serial.println("Received Deactivation");
                sleepTimeout();
                break;
            case HeartbeatMode::CONFIGURE:
                if (configIndex == 0b0001)
                {
                    Serial.println("Received calibrate commands");
                    transitionState(State::CALIBRATE);
                }
                break;
            case HeartbeatMode::ACTIVE:
            case HeartbeatMode::SLOW:
                transitionState(State::ACTIVE);
            }
            break;
        case State::CALIBRATE:
            if (receivedPacketSize == 1)
            {
                parseHeartbeat(received[0]);
                if (configIndex = 0b1111)
                {
                    Serial.println("Received calibration complete");
                    transitionState(State::IDLE);
                }
            }
            else
            {
                x = received[0];
                y = received[1];
                stepper.setSpeed(4 * x);
                stepper2.setSpeed(4 * y);
                if (x == 0 and y == 0)
                {
                    if (stepper.currentPosition() < xMin)
                    {
                        xMin = stepper.currentPosition();
                    }
                    else if (stepper.currentPosition() > xMax)
                    {
                        xMax = stepper.currentPosition();
                    }
                    if (stepper2.currentPosition() < yMin)
                    {
                        yMin = stepper2.currentPosition();
                    }
                    else if (stepper2.currentPosition() > yMax)
                    {
                        yMax = stepper2.currentPosition();
                    }
                    Serial.print("X: ");
                    Serial.print(xMin);
                    Serial.print(" - ");
                    Serial.print(xMax);
                    Serial.print(" Y: ");
                    Serial.print(yMin);
                    Serial.print(" - ");
                    Serial.println(yMax);
                }
            }
        }
    }
}



void setup()
{
    Serial.begin(9600);

    Serial.println("LoRa Receiver");

    pinMode(LED_BUILTIN, OUTPUT);

    //Configure LoRa
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
    LoRa.setTxPower(20);
    LoRa.onReceive(onReceive);
    LoRa.receive();

    //Configure Steppers
    stepper.setMaxSpeed(500);
    stepper2.setMaxSpeed(500);
    stepper.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);

    //Configure Async timers
    heartbeatTimer = t.setInterval(sendHeartbeat, 5000);
}

void loop()
{
    handleMessage();
    handleState();
    t.handle();
}

