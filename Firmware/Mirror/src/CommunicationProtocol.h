#pragma once

#define DEBUG

#include <Arduino.h>

#define ModeMask 0b11 << 6
#define HeartbeatIndexMask 0b11 << 4
#define DataNibbleMask 0b1111

class CommunicationProtocol
{
public:
    enum class HeartbeatMode
    {
        INACTIVE,
        ACTIVE,
        SLOW,
        CONFIGURE,
    };
    
    byte getHeartbeat();
    void parseHeartbeat(byte heartbeat);

    void setHeartbeatMode(HeartbeatMode mode);
    HeartbeatMode getHeartbeatMode();
    void setHeartbeatIndex(int index);
    int getHeartbeatIndex();
    void setHeartbeatData(word data);
    word getHeartbeatData();
    void setHeartbeatConfigIndex(int index);
    int getHeartbeatConfigIndex();

    HeartbeatMode getRemoteHeartbeatMode();
    int getRemoteHeartbeatIndex();
    word getRemoteHeartbeatData();
    int getRemoteHeartbeatConfigIndex();

    word getControlPacket(int x, int y);
    word getConfigPacket(int configValue, int configIndex);

private:
    byte statusByte = 0;
    byte remoteStatusByte = 0;
    HeartbeatMode heartbeatMode = HeartbeatMode::INACTIVE;
    HeartbeatMode remoteHeartbeatMode = HeartbeatMode::INACTIVE;
    byte heartbeatNibble[4] = {0, 0, 0, 0};
    byte remoteHeartbeatNibble[4] = {0, 0, 0, 0};
    int heartbeatIndex = 0;
    int remoteHeartbeatIndex = 0;
    int configIndex = 0;
    int remoteConfigIndex = 0;
    word heartbeatData = 0;
    word remoteHeartbeatData = 0;
};