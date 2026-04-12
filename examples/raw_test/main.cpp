// Minimal RS485 RX test — bypasses the VEBus library entirely.
// If you see hex bytes scrolling, hardware is OK.
// If nothing appears, the RS485 connection is broken.

#include <Arduino.h>
#include "driver/uart.h"

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Raw VE.Bus RX test ===");

    // Reset GPIO 19 and 9 to INPUT in case previous firmware left them as OUTPUT
    pinMode(19, INPUT);
    pinMode(9, INPUT);

    // Same 3 lines as original init_vebus()
    Serial1.begin(256000, SERIAL_8N1, 21, 22);
    Serial1.setPins(-1, -1, -1, 17);
    Serial1.setMode(UART_MODE_RS485_HALF_DUPLEX);

    Serial.println("Listening for RS485 data...");
}

int col = 0;

void loop()
{
    while (Serial1.available())
    {
        uint8_t b = Serial1.read();
        Serial.printf("%02X ", b);
        col++;
        if (b == 0xFF || col >= 32) { Serial.println(); col = 0; }
    }
}
