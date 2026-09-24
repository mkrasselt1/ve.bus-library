/**
 * ups_server — minimal network UPS for a Victron Multiplus
 *
 * Serves the Multiplus as a UPS over NUT (TCP 3493) and apcupsd (TCP 3551),
 * using the optional VEBusUps / VEBusNutServer / VEBusApcupsdServer headers.
 * No MQTT, no web UI — see the mqtt_ha example for the full firmware.
 *
 *   NUT clients:     MONITOR multiplus@<device-ip> 1 user pass secondary
 *   apcupsd clients: apcaccess -h <device-ip>
 *
 * Hardware: LilyGo T-CAN485 (ESP32, MAX13487E RS485 transceiver)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <VEBus.h>
#include <VEBusScaler.h>
#include <VEBusUps.h>
#include <VEBusNutServer.h>
#include <VEBusApcupsdServer.h>

#define WIFI_SSID     "your-ssid"
#define WIFI_PASS     "your-password"

// T-CAN485 pins
#define VEBUS_PIN_RX   21
#define VEBUS_PIN_TX   22
#define VEBUS_PIN_RE   17
#define VEBUS_PIN_SHDN 19

VEBus              vebus;
VEBusScaler        scaler(vebus);   // raw RAM values → V / A / W / % / Hz
VEBusUps           ups(vebus);
VEBusNutServer     nut(ups);
VEBusApcupsdServer apc(ups);

void setup()
{
    Serial.begin(115200);
    pinMode(VEBUS_PIN_SHDN, OUTPUT);
    digitalWrite(VEBUS_PIN_SHDN, LOW);          // transceiver off until UART is ready
    vebus.begin(VEBUS_PIN_RX, VEBUS_PIN_TX, VEBUS_PIN_RE);
    digitalWrite(VEBUS_PIN_SHDN, HIGH);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(250);
    Serial.printf("UPS server at %s\n", WiFi.localIP().toString().c_str());
    configTime(0, 0, "pool.ntp.org");          // timestamps for apcupsd

    ups.lowSocPct = 20;      // report low battery at 20 % SoC
    ups.nominalW  = 2400;    // MultiPlus 3000 VA → load %
    ups.batteryWh = 5000;    // usable battery energy → runtime estimate

    nut.begin();
    apc.begin();
}

void loop()
{
    static uint32_t last = 0;

    // Every 2 s: read what the UPS view needs (the reply arrives asynchronously)
    if (millis() - last >= 2000) {
        last = millis();
        const uint8_t ids[] = {
            VEBUS_RAM_UMAINS_RMS, VEBUS_RAM_UINVERTER_RMS, VEBUS_RAM_OUTPUT_POWER,
            VEBUS_RAM_IBAT, VEBUS_RAM_CHARGE_STATE, VEBUS_RAM_MAINS_PERIOD
        };
        vebus.readRAMVars(ids, 6);
        vebus.requestReadRAM();                // battery voltage
        if (vebus.hasNoSync()) vebus.requestWakeup();
    }

    scaler.loop();       // asks the Multiplus for scale/offset once
    if (vebus.hasRAMVarResponse()) {
        for (uint8_t i = 0; i < vebus.getRAMVarCount(); i++) {
            uint8_t id  = vebus.getRAMVarId(i);
            int16_t raw = vebus.getRAMVarValue(i);
            ups.set(id, id == VEBUS_RAM_MAINS_PERIOD ? scaler.frequency(id, raw)
                                                     : scaler.value(id, raw));
        }
        vebus.clearRAMVarResponse();
    }

    ups.loop();
    nut.loop();
    apc.loop();
}
