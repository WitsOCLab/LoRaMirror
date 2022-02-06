#include <Arduino.h>
#include <SPI.h>

#include "CommunicationProtocol.h"

CommunicationProtocol comms;

void setup()
{
    Serial.begin(115200);
    delay(5000);
    comms.parseHeartbeat(0b01000001);
    comms.parseHeartbeat(0b01010010);
    comms.parseHeartbeat(0b10100011);
    comms.parseHeartbeat(0b00110100);
    Serial.println(comms.getRemoteHeartbeatData(), HEX);
    comms.parseHeartbeat(0b11010010);
    comms.getControlPacket(0, 0);
    comms.getControlPacket(100,100);
    comms.getControlPacket(200,200);
    comms.getControlPacket(-100,-100);
    comms.getConfigPacket(0, 0);
    comms.getConfigPacket(1, 1);
    comms.getConfigPacket(200, 1);
}

void loop()
{
}
