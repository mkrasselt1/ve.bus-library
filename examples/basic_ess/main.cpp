/**
 * basic_ess — VEBus library example for the LilyGo T-CAN485 board
 *
 * The VEBus library runs its own tight-loop task on core 1 (no delay),
 * matching the original PepeTheFroggie architecture where the RS485
 * handler ran in Arduino loop().  This example's loop() is free for
 * application logic on core 1 (the default Arduino core).
 *
 * Hardware: LilyGo T-CAN485 (ESP32 Dev Module)
 *   RS485 RX  → GPIO 21
 *   RS485 TX  → GPIO 22
 *   RS485 DE  → GPIO 17
 *
 * Source:  https://github.com/PepeTheFroggie/Victron-VE.Bus---esp32
 */

#include <Arduino.h>
#include <VEBus.h>

// -----------------------------------------------------------------------
// Pin definitions — LilyGo T-CAN485
// -----------------------------------------------------------------------
#define VEBUS_PIN_RX   21   // MAX13487E pin 1 (RO)
#define VEBUS_PIN_TX   22   // MAX13487E pin 4 (DI)
#define VEBUS_PIN_RE   17   // MAX13487E pin 2 (/RE — receiver enable, active-low)
#define VEBUS_PIN_SHDN 19   // MAX13487E pin 3 (/SHDN — LOW=shutdown, MUST be HIGH!)

// -----------------------------------------------------------------------
// ESS setpoint (watts) — controls power exchange with AC-IN (grid side).
//   > 0  →  invert: push watts FROM battery TOWARD grid/AC-IN
//   < 0  →  charge: pull watts FROM grid INTO battery
//   = 0  →  standby: grid pass-through only
// -----------------------------------------------------------------------
volatile int16_t g_essPower = 0;

// -----------------------------------------------------------------------
// Global library instance
// -----------------------------------------------------------------------
VEBus vebus;

// -----------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------
#define ESS_INTERVAL_MS   5000   // send ESS power every 5 s (matches original)
#define RAM_OFFSET_MS      500   // request data 500 ms before next ESS
#define PRINT_INTERVAL_MS 2000   // print status every 2 s

// -----------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== VEBus Basic ESS example ===");
    Serial.printf("Library compiled %s %s\n", __DATE__, __TIME__);

    // Enable the RS485 transceiver (MAX13487E /SHDN must be HIGH!)
    pinMode(VEBUS_PIN_SHDN, OUTPUT);
    digitalWrite(VEBUS_PIN_SHDN, HIGH);

    // Start the library — launches an internal tight-loop task on core 0.
    // GPIO 17 (/RE) is used as RTS in hardware RS485 mode:
    //   idle → LOW → receiver enabled
    //   TX   → HIGH → receiver disabled (auto-direction handles driver)
    vebus.begin(VEBUS_PIN_RX, VEBUS_PIN_TX, VEBUS_PIN_RE);

    Serial.println("VE.Bus driver started (task on core 0).");
    Serial.printf("Arduino loop() running on core %d.\n", xPortGetCoreID());
}

// -----------------------------------------------------------------------
// loop() — application logic, runs on core 1 (default Arduino core)
//
// Queues ESS power + read RAM commands at the correct intervals,
// and prints decoded Multiplus data.  The VEBus task picks up
// queued commands and sends them during the next sync slot.
// -----------------------------------------------------------------------
void loop()
{
    static unsigned long lastESSMs   = 0;
    static unsigned long lastRAMMs   = 0;
    static unsigned long lastPrintMs = 0;
    static bool          ramRequested = false;

    unsigned long now = millis();

    // --- Queue ESS power command every ESS_INTERVAL_MS ---
    if (now - lastESSMs >= ESS_INTERVAL_MS)
    {
        lastESSMs = now;
        vebus.setESSPower(g_essPower);
        ramRequested = false;  // reset so we request data before next ESS
    }

    // --- Queue read RAM 500 ms before next ESS ---
    if (!ramRequested && (now - lastESSMs >= (ESS_INTERVAL_MS - RAM_OFFSET_MS)))
    {
        vebus.requestReadRAM();
        ramRequested = true;
    }

    // --- Auto-wakeup if no sync ---
    if (vebus.hasNoSync())
    {
        static unsigned long lastWakeupMs = 0;
        if (now - lastWakeupMs >= 3000)
        {
            lastWakeupMs = now;
            vebus.requestWakeup();
            Serial.println("[app] No sync — queued wakeup");
        }
    }

    // --- Print status every 2 s ---
    if (now - lastPrintMs >= PRINT_INTERVAL_MS)
    {
        lastPrintMs = now;

        if (vebus.hasNewData())
            vebus.clearNewData();

        Serial.println("--- Multiplus status ---");
        Serial.printf("  Battery voltage : %.2f V\n",  vebus.getBatVolt());
        Serial.printf("  AC power        : %d W\n",    vebus.getACPower());
        Serial.printf("  DC current      : %.1f A\n",  vebus.getDCCurrent());
        Serial.printf("  Temperature     : %.1f C\n",  vebus.getTemp());
        Serial.printf("  ESS setpoint    : %d W\n",    (int)g_essPower);

        byte leds = vebus.getLEDon();
        Serial.printf("  LEDs on/blink   : 0x%02X / 0x%02X", leds, vebus.getLEDblink());
        if (leds & VEBUS_LED_MAINS_ON)    Serial.print("  [MAINS]");
        if (leds & VEBUS_LED_BULK)        Serial.print("  [BULK]");
        if (leds & VEBUS_LED_ABSORPTION)  Serial.print("  [ABS]");
        if (leds & VEBUS_LED_FLOAT)       Serial.print("  [FLOAT]");
        if (leds & VEBUS_LED_INVERTER_ON) Serial.print("  [INV]");
        if (leds & VEBUS_LED_OVERLOAD)    Serial.print("  [OVERLOAD!]");
        if (leds & VEBUS_LED_LOW_BATTERY) Serial.print("  [LOW BAT!]");
        Serial.println();

        Serial.printf("  AC input limits : min=%.1f A  max=%.1f A  act=%.1f A\n",
                      vebus.getMinInputCurrentLimit(),
                      vebus.getMaxInputCurrentLimit(),
                      vebus.getActInputCurrentLimit());

        if (vebus.getChecksumFaults() > 0)
            Serial.printf("  Checksum faults : %lu\n", vebus.getChecksumFaults());

        Serial.println();
    }

    // --- Serial Monitor commands ---
    // Type a number + Enter → set ESS power (e.g. "300" or "-500")
    // Type "w" + Enter      → wake up Multiplus
    // Type "s" + Enter      → sleep Multiplus
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("w"))
        {
            vebus.requestWakeup();
            Serial.println("[app] Wakeup sent");
        }
        else if (input.equalsIgnoreCase("s"))
        {
            vebus.requestSleep();
            Serial.println("[app] Sleep sent");
        }
        else if (input.length() > 0)
        {
            int16_t newPower = (int16_t)input.toInt();
            newPower = constrain(newPower, -1875, 1875);
            g_essPower = newPower;
            Serial.printf("[app] ESS setpoint → %d W\n", (int)g_essPower);
        }
    }
}
