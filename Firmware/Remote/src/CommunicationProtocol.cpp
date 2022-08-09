#include "CommunicationProtocol.h"

byte CommunicationProtocol::getHeartbeat()
{
    // If the heartbeat index is 0 refresh the data word
    if (heartbeatIndex == 0)
    {
        word tempData = heartbeatData;
        for (int i = 3; i >= 0; i--)
        {
            heartbeatNibble[i] = tempData & DataNibbleMask;
            tempData >>= 4;
        }
    }

    // Then assemble the heartbeat
    statusByte = (byte(heartbeatMode) << 6) & ModeMask;
    statusByte |= (heartbeatIndex << 4) & HeartbeatIndexMask;
    if (heartbeatMode != HeartbeatMode::CONFIGURE)
    {
        statusByte |= heartbeatNibble[heartbeatIndex] & DataNibbleMask;
    }
    else
    {
        statusByte |= configIndex & DataNibbleMask;
    }

    // Finally update the index
    heartbeatIndex++;
    if (heartbeatIndex > 3)
    {
        heartbeatIndex = 0;
    }

#ifdef DEBUG
    Serial.print("Heartbeat sent: ");
    Serial.println(statusByte, BIN);
#endif

    return statusByte;
}

void CommunicationProtocol::parseHeartbeat(byte heartbeat)
{
    remoteHeartbeatMode = (HeartbeatMode)((heartbeat & ModeMask) >> 6);
    remoteHeartbeatIndex = (heartbeat & HeartbeatIndexMask) >> 4;
#ifdef DEBUG
    Serial.print("Heartbeat received: ");
    Serial.print("Mode: ");
    Serial.print(int(remoteHeartbeatMode));
    Serial.print(" Index: ");
    Serial.print(remoteHeartbeatIndex);
#endif
    if (remoteHeartbeatMode != HeartbeatMode::CONFIGURE)
    {
        remoteHeartbeatNibble[remoteHeartbeatIndex] = heartbeat & DataNibbleMask;
#ifdef DEBUG
        Serial.print(" Data: ");
        Serial.println(remoteHeartbeatNibble[remoteHeartbeatIndex], BIN);
#endif
        if (remoteHeartbeatIndex != expectedRemoteHeartbeatIndex)
        {
#ifdef DEBUG
            Serial.print("Heartbeat index mismatch: ");
            Serial.print(remoteHeartbeatIndex);
            Serial.print(" != ");
            Serial.println(expectedRemoteHeartbeatIndex);
#endif
            expectedRemoteHeartbeatIndex = remoteHeartbeatIndex;
            badData = true;
        }
        if (remoteHeartbeatIndex >= 3)
        {
            remoteHeartbeatData = 0;
            expectedRemoteHeartbeatIndex = 0;
            for (int i = 0; i <= 3; i++)
            {
                remoteHeartbeatData <<= 4;
                remoteHeartbeatData |= remoteHeartbeatNibble[i];
            }
        }
        else if (remoteHeartbeatIndex == 0)
        {
            expectedRemoteHeartbeatIndex = 1;
            badData = false;
        }
        else
        {
            expectedRemoteHeartbeatIndex = remoteHeartbeatIndex + 1;
        }
    }
    else
    {
        remoteConfigIndex = heartbeat & DataNibbleMask;

#ifdef DEBUG
        Serial.print(" Config Index: ");
        Serial.println(remoteConfigIndex);
#endif
    }
}

void CommunicationProtocol::setHeartbeatMode(HeartbeatMode mode)
{
    heartbeatMode = mode;
}

CommunicationProtocol::HeartbeatMode CommunicationProtocol::getHeartbeatMode()
{
    return heartbeatMode;
}

void CommunicationProtocol::setHeartbeatIndex(int index)
{
    heartbeatIndex = index;
}

int CommunicationProtocol::getHeartbeatIndex()
{
    return heartbeatIndex;
}

void CommunicationProtocol::setHeartbeatData(word data)
{
    heartbeatData = data;
}

word CommunicationProtocol::getHeartbeatData()
{
    if (badData)
    {
#ifdef DEBUG
        Serial.println("Bad data");
#endif
        return 0;
    }
    else
    {
        return heartbeatData;
    }
}

void CommunicationProtocol::setHeartbeatConfigIndex(int index)
{
    configIndex = index;
}

int CommunicationProtocol::getHeartbeatConfigIndex()
{
    return configIndex;
}

CommunicationProtocol::HeartbeatMode CommunicationProtocol::getRemoteHeartbeatMode()
{
    return remoteHeartbeatMode;
}

int CommunicationProtocol::getRemoteHeartbeatIndex()
{
    return remoteHeartbeatIndex;
}

int CommunicationProtocol::getRemoteHeartbeatConfigIndex()
{
    return remoteConfigIndex;
}

word CommunicationProtocol::getRemoteHeartbeatData()
{
    return remoteHeartbeatData;
}

word CommunicationProtocol::getControlPacket(int x, int y)
{
    // Limit the value range
    if (x > 100)
    {
        x = 100;
    }
    else if (x < -100)
    {
        x = -100;
    }

    if (y > 100)
    {
        y = 100;
    }
    else if (y < -100)
    {
        y = -100;
    }
    word packet = char(x) << 8;
    packet |= char(y);

#ifdef DEBUG
    Serial.print("Control packet sent: ");
    Serial.println(packet, BIN);
#endif
    return packet;
}

word CommunicationProtocol::getConfigPacket(int configValue, int configIndex)
{
    setHeartbeatMode(HeartbeatMode::CONFIGURE);
    setHeartbeatConfigIndex(configIndex);

    word packet = getHeartbeat() << 8;

    if (configValue > 127)
    {
        configValue = 127;
    }
    else if (configValue < -128)
    {
        configValue = -128;
    }
    packet |= char(configValue);

#ifdef DEBUG
    Serial.print("Config packet sent: ");
    Serial.println(packet, BIN);
#endif

    return packet;
}