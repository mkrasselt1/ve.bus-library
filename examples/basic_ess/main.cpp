/**
 * basic_ess — VEBus library example for the LilyGo T-CAN485 board
 *
 * Demonstrates:
 *  - Initialising the VEBus library on the T-CAN485 RS485 port
 *  - Running VE.Bus communication in a dedicated FreeRTOS task on core 0
 *    (matching the original architecture from PepeTheFroggie's sketch)
 *  - Sending an ESS power setpoint each sync cycle
 *  - Requesting and printing battery voltage + AC power
 *
 * Hardware: LilyGo T-CAN485 (ESP32 Dev Module)
 *   RS485 RX  → GPIO 21
 *   RS485 TX  → GPIO 22
 *   RS485 DE  → GPIO 17
 *   (GPIO 19 = RS485 /RE, tie to DE or leave floating if transceiver
 *    handles it internally — check your board revision)
 *
 * Wiring to Victron Multiplus MK3-USB/VE.Bus RS485 adapter:
 *   A (+) → RS485 A
 *   B (−) → RS485 B
 *   GND   → GND  (important!)
 *
 * Protocol note:
 *   The Multiplus broadcasts a sync frame every ~20 ms.
 *   After isSynced() returns true you have ~8 ms before the next frame
 *   to place your command.  The VEBus task below honours this timing.
 *
 * Source:  https://github.com/PepeTheFroggie/Victron-VE.Bus---esp32
 */

#include <Arduino.h>
#include <VEBus.h>

// -----------------------------------------------------------------------
// Pin definitions — LilyGo T-CAN485
// -----------------------------------------------------------------------
#define VEBUS_PIN_RX   21
#define VEBUS_PIN_TX   22
#define VEBUS_PIN_DE   17

// -----------------------------------------------------------------------
// Timing constants
// -----------------------------------------------------------------------
#define TX_SLOT_DELAY_MS   8     // ms after sync before we transmit
#define DATA_REQUEST_MS  500     // request new data every 500 ms

// -----------------------------------------------------------------------
// ESS setpoint (watts) — controls power exchange with AC-IN (grid side).
//   > 0  →  invert: push watts FROM battery TOWARD grid/AC-IN
//            (reduces grid import, or exports to grid if loads are low)
//   < 0  →  charge: pull watts FROM grid INTO battery
//   = 0  →  standby: grid pass-through only
//
// AC-OUT loads are served by grid pass-through + inverter combined.
// This setpoint does NOT directly equal the AC-OUT power to your loads.
//
// Change this at runtime from your application code (mark volatile so
// both cores see the update immediately).
// -----------------------------------------------------------------------
volatile int16_t g_essPower = 0;

// -----------------------------------------------------------------------
// Global library object
// -----------------------------------------------------------------------
VEBus vebus;

// -----------------------------------------------------------------------
// VE.Bus task — runs on core 0, handles all RS485 traffic
//
// This mirrors the original architecture: the time-critical RS485
// communication lives on core 0 so it is not interrupted by WiFi / BT
// activity which shares core 1 with the Arduino loop().
// -----------------------------------------------------------------------
static TaskHandle_t s_vebusTask = nullptr;

static void vebusTaskFn(void *param)
{
    (void)param;

    unsigned long lastDataRequestMs = 0;
    bool          pendingESSWrite   = false;
    bool          pendingDataRead   = false;

    Serial.printf("[VEBus task] running on core %d\n", xPortGetCoreID());

    while (true)
    {
        // --- 1. Process all incoming bytes ---------------------------------
        vebus.handle();

        // --- 2. Schedule periodic requests ---------------------------------
        if (millis() - lastDataRequestMs >= DATA_REQUEST_MS)
        {
            lastDataRequestMs = millis();
            pendingESSWrite   = true;   // send setpoint
            pendingDataRead   = true;   // then read back bat voltage + AC power
        }

        // --- 3. Transmit inside the sync timeslot --------------------------
        //
        // isSynced() is true for exactly one handle() pass after the sync
        // frame's 0xFF terminator is received.  Subsequent bytes clear it,
        // so we must act here, inside this tight loop.
        if (vebus.isSynced() &&
            millis() >= vebus.getSyncTime() + TX_SLOT_DELAY_MS)
        {
            vebus.clearSync();

            if (pendingESSWrite)
            {
                vebus.sendESSPower(g_essPower);
                pendingESSWrite = false;
            }
            else if (pendingDataRead)
            {
                vebus.sendReadRAM();
                pendingDataRead = false;
            }
        }

        // --- 4. No-sync watchdog with automatic wakeup recovery ------------
        //
        // If the Multiplus stops sending sync frames for > 1 s it is likely
        // sleeping.  Send a wakeup command every WAKEUP_RETRY_MS until sync
        // resumes.  isSynced() becoming true again means the device is back.
        if (vebus.hasNoSync())
        {
            static unsigned long lastWakeupAttemptMs = 0;
            const unsigned long WAKEUP_RETRY_MS = 3000;

            if (millis() - lastWakeupAttemptMs >= WAKEUP_RETRY_MS)
            {
                lastWakeupAttemptMs = millis();
                vebus.sendWakeup();
                Serial.println("[VEBus] No sync — sent wakeup, retrying...");
            }
        }

        // Yield to other tasks for 1 ms — keeps CPU use reasonable while
        // still leaving plenty of time to catch every sync frame.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -----------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== VEBus Basic ESS example ===");
    Serial.printf("Library compiled %s %s\n", __DATE__, __TIME__);

    // Initialise the VE.Bus driver (256 kbaud RS485, half-duplex)
    vebus.begin(VEBUS_PIN_RX, VEBUS_PIN_TX, VEBUS_PIN_DE);
    Serial.println("VE.Bus driver initialised.");

    // Start the VE.Bus task on core 0, stack 4 kB, priority 2
    xTaskCreatePinnedToCore(
        vebusTaskFn,    // task function
        "VEBus",        // name
        4096,           // stack size (bytes)
        nullptr,        // parameter
        2,              // priority (higher = more urgent)
        &s_vebusTask,   // handle
        0               // core 0
    );

    Serial.println("VE.Bus task started on core 0.");
    Serial.println("Arduino loop() running on core 1 — printing data every 2 s.");
}

// -----------------------------------------------------------------------
// loop() — runs on core 1
//
// Application code lives here.  Read the decoded values from the shared
// VEBus object and act on them.  The VEBus task on core 0 keeps the
// RS485 traffic alive independently.
//
// Thread safety: the primitive types updated by the VEBus task
// (float, int16_t) are read atomically on Xtensa, so no mutex is needed
// for display purposes.  If you calculate a setpoint here and write it
// to g_essPower, declare it volatile (done above) so the compiler does
// not cache the value in a register.
// -----------------------------------------------------------------------
void loop()
{
    static unsigned long lastPrintMs = 0;

    if (millis() - lastPrintMs >= 2000)
    {
        lastPrintMs = millis();

        Serial.println("--- Multiplus status ---");
        Serial.printf("  Battery voltage : %.2f V\n",  vebus.getBatVolt());
        Serial.printf("  AC power        : %d W\n",    vebus.getACPower());
        Serial.printf("  DC current      : %.1f A\n",  vebus.getDCCurrent());
        Serial.printf("  Temperature     : %.1f °C\n", vebus.getTemp());
        Serial.printf("  ESS setpoint    : %d W\n",    (int)g_essPower);

        // Decode LED bitmask
        byte leds   = vebus.getLEDon();
        byte blinks = vebus.getLEDblink();
        Serial.printf("  LEDs on/blink   : 0x%02X / 0x%02X", leds, blinks);
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

    // -----------------------------------------------------------------
    // Example: simple manual setpoint control via Serial Monitor.
    // Type a wattage (e.g. "300" or "-500") and press Enter.
    // -----------------------------------------------------------------
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0)
        {
            int16_t newPower = (int16_t)input.toInt();
            // Clamp to safe range (±1875 W for a Multiplus 24/3000)
            newPower = constrain(newPower, -1875, 1875);
            g_essPower = newPower;
            Serial.printf("[loop] ESS setpoint updated to %d W\n", (int)g_essPower);
        }
    }
}
