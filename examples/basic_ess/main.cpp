/**
 * basic_ess — VEBus library example for the LilyGo T-CAN485 board
 *
 * Demonstrates all library features via serial console commands.
 *
 * Hardware: LilyGo T-CAN485 (ESP32 Dev Module)
 *   RS485 RX  → GPIO 21
 *   RS485 TX  → GPIO 22
 *   RS485 DE  → GPIO 17
 *
 * Source:  https://github.com/mkrasselt1/ve.bus-library
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
#define ESS_INTERVAL_MS   5000   // send ESS power every 5 s
#define RAM_OFFSET_MS      500   // request data 500 ms before next ESS
#define PRINT_INTERVAL_MS 2000   // print status every 2 s

// -----------------------------------------------------------------------
// Device state name helper
// -----------------------------------------------------------------------
static const char *deviceStateName(uint8_t state)
{
    switch (state) {
    case VEBUS_STATE_DOWN:         return "Down";
    case VEBUS_STATE_STARTUP:      return "Startup";
    case VEBUS_STATE_OFF:          return "Off";
    case VEBUS_STATE_SLAVE:        return "Slave";
    case VEBUS_STATE_INVERT_FULL:  return "Invert Full";
    case VEBUS_STATE_INVERT_HALF:  return "Invert Half";
    case VEBUS_STATE_INVERT_AES:   return "Invert AES";
    case VEBUS_STATE_POWER_ASSIST: return "Power Assist";
    case VEBUS_STATE_BYPASS:       return "Bypass";
    case VEBUS_STATE_CHARGE:       return "Charge";
    default:                       return "Unknown";
    }
}

static const char *chargeSubStateName(uint8_t sub)
{
    switch (sub) {
    case VEBUS_CHARGE_INIT:                return "Init";
    case VEBUS_CHARGE_BULK:                return "Bulk";
    case VEBUS_CHARGE_ABSORPTION:          return "Absorption";
    case VEBUS_CHARGE_FLOAT:               return "Float";
    case VEBUS_CHARGE_STORAGE:             return "Storage";
    case VEBUS_CHARGE_REPEATED_ABSORPTION: return "Repeat Abs";
    case VEBUS_CHARGE_FORCED_ABSORPTION:   return "Forced Abs";
    case VEBUS_CHARGE_EQUALISE:            return "Equalise";
    case VEBUS_CHARGE_BULK_STOPPED:        return "Bulk Stopped";
    default:                               return "Unknown";
    }
}

// -----------------------------------------------------------------------
// Print help
// -----------------------------------------------------------------------
void printHelp()
{
    Serial.println("=== VEBus Serial Commands ===");
    Serial.println("  <number>   Set ESS power (-1875..1875 W)");
    Serial.println("  w          Wakeup (switch ON)");
    Serial.println("  s          Sleep (switch OFF)");
    Serial.println("  co         Charger only");
    Serial.println("  io         Inverter only");
    Serial.println("  v          Request firmware version");
    Serial.println("  d          Request device state");
    Serial.println("  r          Read extended RAM vars (mains V/A, inv V/A, output W)");
    Serial.println("  fa         Force absorption");
    Serial.println("  ff         Force float");
    Serial.println("  fe         Force equalise");
    Serial.println("  rs <id>    Read setting (e.g. rs 2 = absorption voltage)");
    Serial.println("  ws <id> <val>  Write setting (e.g. ws 6 160 = AC limit 16.0A)");
    Serial.println("  ri <id>    Request RAM var info (scale/offset)");
    Serial.println("  si <id>    Request setting info (scale/offset/default/min/max)");
    Serial.println("  h          Show this help");
    Serial.println();
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

    // Enable the RS485 transceiver (MAX13487E /SHDN must be HIGH!)
    pinMode(VEBUS_PIN_SHDN, OUTPUT);
    digitalWrite(VEBUS_PIN_SHDN, HIGH);

    // Start the library — launches an internal tight-loop task on core 0.
    vebus.begin(VEBUS_PIN_RX, VEBUS_PIN_TX, VEBUS_PIN_RE);

    Serial.println("VE.Bus driver started (task on core 0).");
    Serial.printf("Arduino loop() running on core %d.\n\n", xPortGetCoreID());
    printHelp();
}

// -----------------------------------------------------------------------
// loop()
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
        ramRequested = false;
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

    // --- Check for extended RAM variable responses ---
    if (vebus.hasRAMVarResponse())
    {
        vebus.clearRAMVarResponse();
        Serial.println("--- Extended RAM values ---");
        const char *names[] = {"Mains V", "Mains A", "Inv V", "Inv A", "Output W", "SoC"};
        for (uint8_t i = 0; i < vebus.getRAMVarCount(); i++)
        {
            const char *name = (i < 6) ? names[i] : "?";
            Serial.printf("  %s : %d (raw)\n", name, vebus.getRAMVarValue(i));
        }
        Serial.println();
    }

    // --- Check for setting responses ---
    if (vebus.hasSettingResponse())
    {
        Serial.printf("[resp] Setting %d = %u (0x%04X)\n",
                      vebus.getSettingId(), vebus.getSettingValue(), vebus.getSettingValue());
        vebus.clearSettingResponse();
    }

    if (vebus.isSettingWriteAcked())
    {
        Serial.println("[resp] Setting write acknowledged");
        vebus.clearSettingWriteAcked();
    }

    // --- Check for device state responses ---
    if (vebus.hasDeviceStateResponse())
    {
        uint8_t st  = vebus.getDeviceState();
        uint8_t sub = vebus.getDeviceSubState();
        Serial.printf("[resp] Device state: %s (%d)", deviceStateName(st), st);
        if (st == VEBUS_STATE_CHARGE)
            Serial.printf("  sub-state: %s (%d)", chargeSubStateName(sub), sub);
        Serial.println();
        vebus.clearDeviceStateResponse();
    }

    // --- Check for version responses ---
    if (vebus.hasVersionResponse())
    {
        Serial.printf("[resp] Firmware version: %u.%u (0x%04X 0x%04X)\n",
                      vebus.getVersionHigh(), vebus.getVersionLow(),
                      vebus.getVersionHigh(), vebus.getVersionLow());
        vebus.clearVersionResponse();
    }

    // --- Check for setting info responses ---
    if (vebus.hasSettingInfoResponse())
    {
        const VEBusSettingInfo &info = vebus.getSettingInfo();
        Serial.printf("[resp] Setting %d info: scale=%d offset=%d default=%u min=%u max=%u\n",
                      info.id, info.scale, info.offset,
                      info.defaultValue, info.minimum, info.maximum);
        vebus.clearSettingInfoResponse();
    }

    // --- Check for RAM var info responses ---
    if (vebus.hasRAMVarInfoResponse())
    {
        Serial.printf("[resp] RAM var %d info: scale=%d offset=%d\n",
                      vebus.getRAMVarInfoId(), vebus.getRAMVarInfoScale(),
                      vebus.getRAMVarInfoOffset());
        vebus.clearRAMVarInfoResponse();
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

        byte sw = vebus.getSwitchRegister();
        if (sw) {
            Serial.printf("  Switch register : 0x%02X", sw);
            if (sw & VEBUS_SWITCH_CHARGE)           Serial.print(" [CHG]");
            if (sw & VEBUS_SWITCH_INVERT)           Serial.print(" [INV]");
            if (sw & VEBUS_SWITCH_FRONT_UP)         Serial.print(" [FrontUP]");
            if (sw & VEBUS_SWITCH_FRONT_DOWN)       Serial.print(" [FrontDN]");
            if (sw & VEBUS_SWITCH_REMOTE_GENERATOR) Serial.print(" [GEN]");
            Serial.println();
        }

        if (vebus.getChecksumFaults() > 0)
            Serial.printf("  Checksum faults : %lu\n", vebus.getChecksumFaults());

        Serial.println();
    }

    // --- Serial Monitor commands ---
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("h") || input.equalsIgnoreCase("help"))
        {
            printHelp();
        }
        else if (input.equalsIgnoreCase("w"))
        {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_ON);
            Serial.println("[app] Switch ON (wakeup) sent");
        }
        else if (input.equalsIgnoreCase("s"))
        {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_OFF);
            Serial.println("[app] Switch OFF (sleep) sent");
        }
        else if (input.equalsIgnoreCase("co"))
        {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_CHARGER_ONLY);
            Serial.println("[app] Charger-only mode sent");
        }
        else if (input.equalsIgnoreCase("io"))
        {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_INVERTER_ONLY);
            Serial.println("[app] Inverter-only mode sent");
        }
        else if (input.equalsIgnoreCase("v"))
        {
            vebus.requestVersion();
            Serial.println("[app] Version request queued");
        }
        else if (input.equalsIgnoreCase("d"))
        {
            vebus.requestDeviceState();
            Serial.println("[app] Device state request queued");
        }
        else if (input.equalsIgnoreCase("r"))
        {
            // Read extended set: mains V/A, inverter V/A, output power, SoC
            const uint8_t ids[] = {
                VEBUS_RAM_UMAINS_RMS, VEBUS_RAM_IMAINS_RMS,
                VEBUS_RAM_UINVERTER_RMS, VEBUS_RAM_IINVERTER_RMS,
                VEBUS_RAM_OUTPUT_POWER, VEBUS_RAM_CHARGE_STATE
            };
            vebus.readRAMVars(ids, 6);
            Serial.println("[app] Extended RAM read queued (6 vars)");
        }
        else if (input.equalsIgnoreCase("fa"))
        {
            vebus.forceDeviceState(VEBUS_FORCE_ABSORPTION);
            Serial.println("[app] Force absorption sent");
        }
        else if (input.equalsIgnoreCase("ff"))
        {
            vebus.forceDeviceState(VEBUS_FORCE_FLOAT);
            Serial.println("[app] Force float sent");
        }
        else if (input.equalsIgnoreCase("fe"))
        {
            vebus.forceDeviceState(VEBUS_FORCE_EQUALISE);
            Serial.println("[app] Force equalise sent");
        }
        else if (input.startsWith("rs "))
        {
            uint8_t id = (uint8_t)input.substring(3).toInt();
            vebus.readSetting(id);
            Serial.printf("[app] Read setting %d queued\n", id);
        }
        else if (input.startsWith("ws "))
        {
            int spaceIdx = input.indexOf(' ', 3);
            if (spaceIdx > 0) {
                uint8_t id = (uint8_t)input.substring(3, spaceIdx).toInt();
                uint16_t val = (uint16_t)input.substring(spaceIdx + 1).toInt();
                vebus.writeSetting(id, val);
                Serial.printf("[app] Write setting %d = %u queued\n", id, val);
            } else {
                Serial.println("[app] Usage: ws <id> <value>");
            }
        }
        else if (input.startsWith("ri "))
        {
            uint8_t id = (uint8_t)input.substring(3).toInt();
            vebus.requestRAMVarInfo(id);
            Serial.printf("[app] RAM var %d info request queued\n", id);
        }
        else if (input.startsWith("si "))
        {
            uint8_t id = (uint8_t)input.substring(3).toInt();
            vebus.requestSettingInfo(id);
            Serial.printf("[app] Setting %d info request queued\n", id);
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
