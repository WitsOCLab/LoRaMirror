#include <TTN_esp32.h>

#include "TTN_CayenneLPP.h"
/***************************************************************************
 *  Go to your TTN console register a device then the copy fields
 *  and replace the CHANGE_ME strings below
 ****************************************************************************/
const char* devEui = "002CF661A596CF8E"; // Change to TTN Device EUI
const char* appEui = "70B3D57ED003D602"; // Change to TTN Application EUI
const char* appKey = "4479DE992478DB2190FD20D99201AF37"; // Chaneg to TTN Application Key

TTN_esp32 ttn ;
TTN_CayenneLPP lpp;

#define UNUSED_PIN 0xFF
#define SS 18
#define RST_LoRa 14
#define DIO0 26
#define DIO1 33
#define DIO2 32

void message(const uint8_t* payload, size_t size, int rssi)
{
    Serial.println("-- MESSAGE");
    Serial.print("Received " + String(size) + " bytes RSSI=" + String(rssi) + "db");
    for (int i = 0; i < size; i++)
    {
        Serial.print(" " + String(payload[i]));
        // Serial.write(payload[i]);
    }

    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting");
    ttn.begin(SS, UNUSED_PIN, RST_LoRa, DIO0, DIO1, DIO2);
    ttn.onMessage(message); // Declare callback function for handling downlink
                            // messages from server
    ttn.join(devEui, appEui, appKey);
    Serial.print("Joining TTN ");
    while (!ttn.isJoined())
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\njoined !");
    ttn.showStatus();
}

void loop()
{
    static float nb = 18.2;
    nb += 0.1;
    lpp.reset();
    lpp.addTemperature(1, nb);
    if (ttn.sendBytes(lpp.getBuffer(), lpp.getSize()))
    {
        Serial.printf("Temp: %f TTN_CayenneLPP: %d %x %02X%02X\n", nb, lpp.getBuffer()[0], lpp.getBuffer()[1],
            lpp.getBuffer()[2], lpp.getBuffer()[3]);
    }
    delay(10000);
}
