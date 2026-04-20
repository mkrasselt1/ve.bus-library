/**
 * mqtt_ha — VEBus → MQTT → Home Assistant auto-discovery
 *
 * Publishes all Multiplus data as HA sensors and exposes
 * ESS power (number), switch state (select), and buttons as controls.
 *
 * Hardware: LilyGo T-CAN485 (ESP32, MAX13487E RS485 transceiver)
 *
 * Source:  https://github.com/mkrasselt1/ve.bus-library
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <VEBus.h>

// =======================================================================
// Configuration — edit these for your setup
// =======================================================================
const char *WIFI_SSID     = "YOUR_SSID";
const char *WIFI_PASS     = "YOUR_PASSWORD";
const char *MQTT_HOST     = "192.168.1.100";
const uint16_t MQTT_PORT  = 1883;
const char *MQTT_USER     = "";          // leave empty if no auth
const char *MQTT_PASS     = "";
const char *DEVICE_ID     = "vebus_multiplus";
const char *TOPIC_PREFIX  = "vebus/multiplus";

// T-CAN485 pins
#define VEBUS_PIN_RX   21
#define VEBUS_PIN_TX   22
#define VEBUS_PIN_RE   17
#define VEBUS_PIN_SHDN 19

// Timing
#define PUBLISH_INTERVAL_MS  5000
#define ESS_INTERVAL_MS      5000
#define RAM_OFFSET_MS         500
#define EXT_RAM1_OFFSET_MS   1000   // 1st extended RAM read
#define EXT_RAM2_OFFSET_MS   2000   // 2nd extended RAM read
#define STATE_OFFSET_MS      3000   // device state request
#define MQTT_RECONNECT_MS    5000
#define WAKEUP_RETRY_MS      3000

// =======================================================================
// Globals
// =======================================================================
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
VEBus        vebus;

volatile int16_t g_essPower = 0;

// Topic buffers
char topicBuf[128];
char payloadBuf[512];

// Extended RAM batch 1: voltages, currents, power
volatile int16_t g_mainsVoltage    = 0;
volatile int16_t g_mainsCurrent    = 0;
volatile int16_t g_inverterVoltage = 0;
volatile int16_t g_inverterCurrent = 0;
volatile int16_t g_outputPower     = 0;
volatile int16_t g_mainsPower      = 0;

// Extended RAM batch 2: battery, frequency, SoC
volatile int16_t g_batteryCurrent  = 0;
volatile int16_t g_chargeState     = 0;  // SoC raw value
volatile int16_t g_mainsFreq       = 0;  // period raw (Hz = 10/value)
volatile int16_t g_inverterFreq    = 0;  // period raw (Hz = 10/value)

// Track which RAM batch the response belongs to
volatile uint8_t g_lastRamBatch = 0;

// =======================================================================
// Device state name helpers
// =======================================================================
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

// =======================================================================
// Helper: build a full topic from suffix
// =======================================================================
const char *topic(const char *suffix)
{
    snprintf(topicBuf, sizeof(topicBuf), "%s/%s", TOPIC_PREFIX, suffix);
    return topicBuf;
}

// =======================================================================
// HA MQTT auto-discovery: publish config payloads (retained)
// =======================================================================
static const char *DEVICE_JSON =
    "\"dev\":{\"ids\":[\"%s\"],\"name\":\"Victron Multiplus\","
    "\"mfr\":\"Victron Energy\",\"mdl\":\"Multiplus\"}";

void publishSensorConfig(const char *name, const char *uid,
                         const char *unit, const char *devClass,
                         const char *stateTopic)
{
    char configTopic[128];
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/sensor/%s/config", uid);

    int len;
    if (devClass && devClass[0]) {
        len = snprintf(payloadBuf, sizeof(payloadBuf),
            "{\"name\":\"%s\",\"uniq_id\":\"%s\","
            "\"stat_t\":\"%s/%s\","
            "\"unit_of_meas\":\"%s\",\"dev_cla\":\"%s\","
            "\"avty_t\":\"%s/status\","
            "\"expire_after\":30,",
            name, uid, TOPIC_PREFIX, stateTopic,
            unit, devClass, TOPIC_PREFIX);
    } else {
        len = snprintf(payloadBuf, sizeof(payloadBuf),
            "{\"name\":\"%s\",\"uniq_id\":\"%s\","
            "\"stat_t\":\"%s/%s\","
            "\"avty_t\":\"%s/status\","
            "\"expire_after\":30,",
            name, uid, TOPIC_PREFIX, stateTopic, TOPIC_PREFIX);
    }
    snprintf(payloadBuf + len, sizeof(payloadBuf) - len, DEVICE_JSON, DEVICE_ID);
    strlcat(payloadBuf, "}", sizeof(payloadBuf));

    mqtt.publish(configTopic, payloadBuf, true);
}

void publishBinarySensorConfig(const char *name, const char *uid,
                               const char *devClass, const char *stateTopic)
{
    char configTopic[128];
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/binary_sensor/%s/config", uid);

    int len = snprintf(payloadBuf, sizeof(payloadBuf),
        "{\"name\":\"%s\",\"uniq_id\":\"%s\","
        "\"stat_t\":\"%s/%s\","
        "\"dev_cla\":\"%s\","
        "\"avty_t\":\"%s/status\",",
        name, uid, TOPIC_PREFIX, stateTopic,
        devClass, TOPIC_PREFIX);
    snprintf(payloadBuf + len, sizeof(payloadBuf) - len, DEVICE_JSON, DEVICE_ID);
    strlcat(payloadBuf, "}", sizeof(payloadBuf));

    mqtt.publish(configTopic, payloadBuf, true);
}

void publishNumberConfig()
{
    char configTopic[128];
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/number/%s_ess/config", DEVICE_ID);

    int len = snprintf(payloadBuf, sizeof(payloadBuf),
        "{\"name\":\"ESS Power Setpoint\",\"uniq_id\":\"%s_ess\","
        "\"stat_t\":\"%s/ess_power\","
        "\"cmd_t\":\"%s/ess_power/set\","
        "\"min\":-1875,\"max\":1875,\"step\":1,"
        "\"unit_of_meas\":\"W\",\"mode\":\"box\","
        "\"avty_t\":\"%s/status\",",
        DEVICE_ID, TOPIC_PREFIX, TOPIC_PREFIX, TOPIC_PREFIX);
    snprintf(payloadBuf + len, sizeof(payloadBuf) - len, DEVICE_JSON, DEVICE_ID);
    strlcat(payloadBuf, "}", sizeof(payloadBuf));

    mqtt.publish(configTopic, payloadBuf, true);
}

void publishButtonConfig(const char *name, const char *uid,
                         const char *cmdSuffix)
{
    char configTopic[128];
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/button/%s/config", uid);

    int len = snprintf(payloadBuf, sizeof(payloadBuf),
        "{\"name\":\"%s\",\"uniq_id\":\"%s\","
        "\"cmd_t\":\"%s/%s/set\","
        "\"payload_press\":\"PRESS\","
        "\"avty_t\":\"%s/status\",",
        name, uid, TOPIC_PREFIX, cmdSuffix, TOPIC_PREFIX);
    snprintf(payloadBuf + len, sizeof(payloadBuf) - len, DEVICE_JSON, DEVICE_ID);
    strlcat(payloadBuf, "}", sizeof(payloadBuf));

    mqtt.publish(configTopic, payloadBuf, true);
}

void publishSelectConfig()
{
    char configTopic[128];
    snprintf(configTopic, sizeof(configTopic),
             "homeassistant/select/%s_switch/config", DEVICE_ID);

    int len = snprintf(payloadBuf, sizeof(payloadBuf),
        "{\"name\":\"Switch State\",\"uniq_id\":\"%s_switch\","
        "\"stat_t\":\"%s/switch_state\","
        "\"cmd_t\":\"%s/switch_state/set\","
        "\"options\":[\"on\",\"off\",\"charger_only\",\"inverter_only\"],"
        "\"avty_t\":\"%s/status\",",
        DEVICE_ID, TOPIC_PREFIX, TOPIC_PREFIX, TOPIC_PREFIX);
    snprintf(payloadBuf + len, sizeof(payloadBuf) - len, DEVICE_JSON, DEVICE_ID);
    strlcat(payloadBuf, "}", sizeof(payloadBuf));

    mqtt.publish(configTopic, payloadBuf, true);
}

void publishDiscovery()
{
    // --- Sensors: core (from legacy RAM read + broadcast frames) ---
    publishSensorConfig("Battery Voltage",  "vebus_bat_volt",     "V",  "voltage",     "bat_volt");
    publishSensorConfig("AC Power",         "vebus_ac_power",     "W",  "power",       "ac_power");
    publishSensorConfig("DC Current",       "vebus_dc_current",   "A",  "current",     "dc_current");
    publishSensorConfig("Temperature",      "vebus_temp",         "\xC2\xB0""C", "temperature", "temp");
    publishSensorConfig("Charger Status",   "vebus_charger",      "",   "",            "charger_status");
    publishSensorConfig("ESS Power",        "vebus_ess_state",    "W",  "power",       "ess_power");

    // --- Sensors: extended RAM batch 1 (AC voltages, currents, power) ---
    publishSensorConfig("Mains Voltage",    "vebus_mains_v",      "V",  "voltage",     "mains_voltage");
    publishSensorConfig("Mains Current",    "vebus_mains_a",      "A",  "current",     "mains_current");
    publishSensorConfig("Inverter Voltage", "vebus_inv_v",        "V",  "voltage",     "inv_voltage");
    publishSensorConfig("Inverter Current", "vebus_inv_a",        "A",  "current",     "inv_current");
    publishSensorConfig("Output Power",     "vebus_output_w",     "W",  "power",       "output_power");
    publishSensorConfig("Mains Power",      "vebus_mains_w",      "W",  "power",       "mains_power");

    // --- Sensors: extended RAM batch 2 (battery, frequency, SoC) ---
    publishSensorConfig("Battery Current",  "vebus_bat_a",        "A",  "current",     "bat_current");
    publishSensorConfig("State of Charge",  "vebus_soc",          "%",  "battery",     "soc");
    publishSensorConfig("Mains Frequency",  "vebus_mains_hz",     "Hz", "frequency",   "mains_freq");
    publishSensorConfig("Inverter Frequency","vebus_inv_hz",      "Hz", "frequency",   "inv_freq");

    // --- Sensors: LED, limits, status ---
    publishSensorConfig("LED On",           "vebus_led_on",       "",   "",            "led_on");
    publishSensorConfig("LED Blink",        "vebus_led_blink",    "",   "",            "led_blink");
    publishSensorConfig("AC Input Min",     "vebus_ac_in_min",    "A",  "current",     "ac_in_min");
    publishSensorConfig("AC Input Max",     "vebus_ac_in_max",    "A",  "current",     "ac_in_max");
    publishSensorConfig("AC Input Actual",  "vebus_ac_in_actual", "A",  "current",     "ac_in_actual");
    publishSensorConfig("AC Input Config",  "vebus_ac_in_cfg",    "",   "",            "ac_in_config");
    publishSensorConfig("Device State",     "vebus_dev_state",    "",   "",            "device_state");
    publishSensorConfig("Charge Sub-State", "vebus_charge_sub",   "",   "",            "charge_sub_state");
    publishSensorConfig("Checksum Faults",  "vebus_chksum",       "",   "",            "checksum_faults");

    // --- Binary sensors ---
    publishBinarySensorConfig("VE.Bus Sync",          "vebus_sync",     "connectivity", "sync");
    publishBinarySensorConfig("DC Allows Inverting",   "vebus_dc_ok",   "power",        "dc_allows_inv");

    // --- Number input ---
    publishNumberConfig();

    // --- Select: switch state ---
    publishSelectConfig();

    // --- Buttons ---
    publishButtonConfig("Wakeup Multiplus",      "vebus_wakeup",       "wakeup");
    publishButtonConfig("Sleep Multiplus",        "vebus_sleep",        "sleep");
    publishButtonConfig("Force Absorption",       "vebus_force_abs",    "force_absorption");
    publishButtonConfig("Force Float",            "vebus_force_float",  "force_float");
    publishButtonConfig("Force Equalise",         "vebus_force_eq",     "force_equalise");
}

// =======================================================================
// MQTT callback — handle commands from HA
// =======================================================================
void mqttCallback(char *topicStr, byte *payload, unsigned int length)
{
    char msg[32];
    int len = min((unsigned int)31, length);
    memcpy(msg, payload, len);
    msg[len] = '\0';

    if (strstr(topicStr, "ess_power/set"))
    {
        int16_t watts = (int16_t)constrain(atoi(msg), -1875, 1875);
        g_essPower = watts;
        vebus.setESSPower(watts);
        snprintf(payloadBuf, sizeof(payloadBuf), "%d", watts);
        mqtt.publish(topic("ess_power"), payloadBuf);
        Serial.printf("[MQTT] ESS → %d W\n", watts);
    }
    else if (strstr(topicStr, "switch_state/set"))
    {
        String cmd(msg);
        if (cmd == "on") {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_ON);
            mqtt.publish(topic("switch_state"), "on");
        } else if (cmd == "off") {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_OFF);
            mqtt.publish(topic("switch_state"), "off");
        } else if (cmd == "charger_only") {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_CHARGER_ONLY);
            mqtt.publish(topic("switch_state"), "charger_only");
        } else if (cmd == "inverter_only") {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_INVERTER_ONLY);
            mqtt.publish(topic("switch_state"), "inverter_only");
        }
        Serial.printf("[MQTT] Switch → %s\n", msg);
    }
    else if (strstr(topicStr, "sleep/set"))
    {
        vebus.setSwitchState(VEBUS_SWITCH_STATE_OFF);
        mqtt.publish(topic("switch_state"), "off");
        Serial.println("[MQTT] Sleep sent");
    }
    else if (strstr(topicStr, "wakeup/set"))
    {
        vebus.setSwitchState(VEBUS_SWITCH_STATE_ON);
        mqtt.publish(topic("switch_state"), "on");
        Serial.println("[MQTT] Wakeup sent");
    }
    else if (strstr(topicStr, "force_absorption/set"))
    {
        vebus.forceDeviceState(VEBUS_FORCE_ABSORPTION);
        Serial.println("[MQTT] Force absorption sent");
    }
    else if (strstr(topicStr, "force_float/set"))
    {
        vebus.forceDeviceState(VEBUS_FORCE_FLOAT);
        Serial.println("[MQTT] Force float sent");
    }
    else if (strstr(topicStr, "force_equalise/set"))
    {
        vebus.forceDeviceState(VEBUS_FORCE_EQUALISE);
        Serial.println("[MQTT] Force equalise sent");
    }
}

// =======================================================================
// MQTT connect with LWT
// =======================================================================
void connectMqtt()
{
    static unsigned long lastAttempt = 0;
    if (mqtt.connected()) return;
    if (millis() - lastAttempt < MQTT_RECONNECT_MS) return;
    lastAttempt = millis();

    Serial.print("[MQTT] Connecting...");

    char statusTopic[128];
    snprintf(statusTopic, sizeof(statusTopic), "%s/status", TOPIC_PREFIX);

    bool ok;
    if (MQTT_USER[0])
        ok = mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS,
                          statusTopic, 1, true, "offline");
    else
        ok = mqtt.connect(DEVICE_ID,
                          statusTopic, 1, true, "offline");

    if (ok)
    {
        Serial.println(" connected!");
        mqtt.publish(statusTopic, "online", true);

        char subTopic[128];
        snprintf(subTopic, sizeof(subTopic), "%s/+/set", TOPIC_PREFIX);
        mqtt.subscribe(subTopic);

        publishDiscovery();

        // Request firmware version + device state once at connect
        vebus.requestVersion();
        vebus.requestDeviceState();
    }
    else
    {
        Serial.printf(" failed (rc=%d)\n", mqtt.state());
    }
}

// =======================================================================
// Publish all sensor values
// =======================================================================
void publishSensors()
{
    char val[32];

    // --- Core values (legacy RAM read + broadcast frames) ---
    dtostrf(vebus.getBatVolt(), 1, 2, val);
    mqtt.publish(topic("bat_volt"), val);

    snprintf(val, sizeof(val), "%d", vebus.getACPower());
    mqtt.publish(topic("ac_power"), val);

    dtostrf(vebus.getDCCurrent(), 1, 1, val);
    mqtt.publish(topic("dc_current"), val);

    dtostrf(vebus.getTemp(), 1, 1, val);
    mqtt.publish(topic("temp"), val);

    snprintf(val, sizeof(val), "%d", vebus.getChargerStatus());
    mqtt.publish(topic("charger_status"), val);

    snprintf(val, sizeof(val), "%d", (int)g_essPower);
    mqtt.publish(topic("ess_power"), val);

    // --- Extended RAM batch 1: AC voltages, currents, power ---
    snprintf(val, sizeof(val), "%d", (int)g_mainsVoltage);
    mqtt.publish(topic("mains_voltage"), val);

    snprintf(val, sizeof(val), "%d", (int)g_mainsCurrent);
    mqtt.publish(topic("mains_current"), val);

    snprintf(val, sizeof(val), "%d", (int)g_inverterVoltage);
    mqtt.publish(topic("inv_voltage"), val);

    snprintf(val, sizeof(val), "%d", (int)g_inverterCurrent);
    mqtt.publish(topic("inv_current"), val);

    snprintf(val, sizeof(val), "%d", (int)g_outputPower);
    mqtt.publish(topic("output_power"), val);

    snprintf(val, sizeof(val), "%d", (int)g_mainsPower);
    mqtt.publish(topic("mains_power"), val);

    // --- Extended RAM batch 2: battery current, SoC, frequencies ---
    snprintf(val, sizeof(val), "%d", (int)g_batteryCurrent);
    mqtt.publish(topic("bat_current"), val);

    snprintf(val, sizeof(val), "%d", (int)g_chargeState);
    mqtt.publish(topic("soc"), val);

    // Convert period to frequency: Hz = 10 / period (period in 0.1s units)
    if (g_mainsFreq > 0)
        dtostrf(10.0f / (float)g_mainsFreq, 1, 1, val);
    else
        strcpy(val, "0");
    mqtt.publish(topic("mains_freq"), val);

    if (g_inverterFreq > 0)
        dtostrf(10.0f / (float)g_inverterFreq, 1, 1, val);
    else
        strcpy(val, "0");
    mqtt.publish(topic("inv_freq"), val);

    // --- LED & status ---
    snprintf(val, sizeof(val), "%d", vebus.getLEDon());
    mqtt.publish(topic("led_on"), val);

    snprintf(val, sizeof(val), "%d", vebus.getLEDblink());
    mqtt.publish(topic("led_blink"), val);

    dtostrf(vebus.getMinInputCurrentLimit(), 1, 1, val);
    mqtt.publish(topic("ac_in_min"), val);

    dtostrf(vebus.getMaxInputCurrentLimit(), 1, 1, val);
    mqtt.publish(topic("ac_in_max"), val);

    dtostrf(vebus.getActInputCurrentLimit(), 1, 1, val);
    mqtt.publish(topic("ac_in_actual"), val);

    snprintf(val, sizeof(val), "%d", vebus.getAcInputConfiguration());
    mqtt.publish(topic("ac_in_config"), val);

    snprintf(val, sizeof(val), "%lu", vebus.getChecksumFaults());
    mqtt.publish(topic("checksum_faults"), val);

    // --- Binary values ---
    mqtt.publish(topic("sync"), vebus.hasNoSync() ? "OFF" : "ON");
    mqtt.publish(topic("dc_allows_inv"), vebus.dcLevelAllowsInverting() ? "ON" : "OFF");

    // --- Derive switch state from switch register ---
    byte sw = vebus.getSwitchRegister();
    bool chg = sw & VEBUS_SWITCH_CHARGE;
    bool inv = sw & VEBUS_SWITCH_INVERT;
    if (chg && inv)       mqtt.publish(topic("switch_state"), "on");
    else if (chg && !inv) mqtt.publish(topic("switch_state"), "charger_only");
    else if (!chg && inv) mqtt.publish(topic("switch_state"), "inverter_only");
    else                  mqtt.publish(topic("switch_state"), "off");
}

// =======================================================================
// setup()
// =======================================================================
void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== VEBus MQTT → Home Assistant ===");

    // Enable RS485 transceiver
    pinMode(VEBUS_PIN_SHDN, OUTPUT);
    digitalWrite(VEBUS_PIN_SHDN, HIGH);

    // Start VEBus (internal task on core 0)
    vebus.begin(VEBUS_PIN_RX, VEBUS_PIN_TX, VEBUS_PIN_RE);
    Serial.println("VE.Bus started.");

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf(" connected (%s)\n", WiFi.localIP().toString().c_str());

    // MQTT
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setBufferSize(512);
    mqtt.setCallback(mqttCallback);
    connectMqtt();
}

// =======================================================================
// loop()
// =======================================================================
void loop()
{
    static unsigned long lastESSMs     = 0;
    static unsigned long lastPublishMs = 0;
    static bool          ramRequested    = false;
    static bool          extRam1Requested = false;
    static bool          extRam2Requested = false;
    static bool          stateRequested  = false;

    unsigned long now = millis();

    // MQTT keepalive + reconnect
    if (!mqtt.connected()) connectMqtt();
    mqtt.loop();

    // Queue ESS power every ESS_INTERVAL_MS
    if (now - lastESSMs >= ESS_INTERVAL_MS)
    {
        lastESSMs = now;
        vebus.setESSPower(g_essPower);
        ramRequested     = false;
        extRam1Requested = false;
        extRam2Requested = false;
        stateRequested   = false;
    }

    // Queue legacy read RAM (bat voltage + AC power)
    if (!ramRequested && (now - lastESSMs >= (ESS_INTERVAL_MS - RAM_OFFSET_MS)))
    {
        vebus.requestReadRAM();
        ramRequested = true;
    }

    // Queue extended RAM batch 1: mains V/A, inverter V/A, output W, mains W
    if (!extRam1Requested && (now - lastESSMs >= EXT_RAM1_OFFSET_MS))
    {
        const uint8_t ids[] = {
            VEBUS_RAM_UMAINS_RMS, VEBUS_RAM_IMAINS_RMS,
            VEBUS_RAM_UINVERTER_RMS, VEBUS_RAM_IINVERTER_RMS,
            VEBUS_RAM_OUTPUT_POWER, VEBUS_RAM_MAINS_POWER
        };
        vebus.readRAMVars(ids, 6);
        g_lastRamBatch = 1;
        extRam1Requested = true;
    }

    // Queue extended RAM batch 2: battery current, SoC, mains freq, inverter freq
    if (!extRam2Requested && (now - lastESSMs >= EXT_RAM2_OFFSET_MS))
    {
        const uint8_t ids[] = {
            VEBUS_RAM_IBAT, VEBUS_RAM_CHARGE_STATE,
            VEBUS_RAM_MAINS_PERIOD, VEBUS_RAM_INVERTER_PERIOD
        };
        vebus.readRAMVars(ids, 4);
        g_lastRamBatch = 2;
        extRam2Requested = true;
    }

    // Queue device state request
    if (!stateRequested && (now - lastESSMs >= STATE_OFFSET_MS))
    {
        vebus.requestDeviceState();
        stateRequested = true;
    }

    // Process extended RAM responses
    if (vebus.hasRAMVarResponse())
    {
        uint8_t count = vebus.getRAMVarCount();
        if (g_lastRamBatch == 1 && count >= 6)
        {
            g_mainsVoltage    = vebus.getRAMVarValue(0);
            g_mainsCurrent    = vebus.getRAMVarValue(1);
            g_inverterVoltage = vebus.getRAMVarValue(2);
            g_inverterCurrent = vebus.getRAMVarValue(3);
            g_outputPower     = vebus.getRAMVarValue(4);
            g_mainsPower      = vebus.getRAMVarValue(5);
        }
        else if (g_lastRamBatch == 2 && count >= 4)
        {
            g_batteryCurrent = vebus.getRAMVarValue(0);
            g_chargeState    = vebus.getRAMVarValue(1);
            g_mainsFreq      = vebus.getRAMVarValue(2);
            g_inverterFreq   = vebus.getRAMVarValue(3);
        }
        vebus.clearRAMVarResponse();
    }

    // Process device state responses → publish immediately
    if (vebus.hasDeviceStateResponse())
    {
        if (mqtt.connected())
        {
            uint8_t st  = vebus.getDeviceState();
            uint8_t sub = vebus.getDeviceSubState();
            mqtt.publish(topic("device_state"), deviceStateName(st));
            mqtt.publish(topic("charge_sub_state"),
                         (st == VEBUS_STATE_CHARGE) ? chargeSubStateName(sub) : "N/A");
        }
        vebus.clearDeviceStateResponse();
    }

    // Process version responses → publish once (retained)
    if (vebus.hasVersionResponse())
    {
        char val[32];
        snprintf(val, sizeof(val), "%u.%u", vebus.getVersionHigh(), vebus.getVersionLow());
        if (mqtt.connected())
            mqtt.publish(topic("firmware_version"), val, true);
        vebus.clearVersionResponse();
    }

    // Auto-wakeup on no-sync
    if (vebus.hasNoSync())
    {
        static unsigned long lastWakeupMs = 0;
        if (now - lastWakeupMs >= WAKEUP_RETRY_MS)
        {
            lastWakeupMs = now;
            vebus.requestWakeup();
            Serial.println("[app] No sync — queued wakeup");
        }
    }

    // Publish all sensor values periodically
    if (now - lastPublishMs >= PUBLISH_INTERVAL_MS)
    {
        lastPublishMs = now;
        if (vebus.hasNewData()) vebus.clearNewData();
        if (mqtt.connected()) publishSensors();
    }

    // Serial console: w=wake, s=sleep, number=ESS watts
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("w"))
        {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_ON);
            Serial.println("[app] Wakeup sent");
        }
        else if (input.equalsIgnoreCase("s"))
        {
            vebus.setSwitchState(VEBUS_SWITCH_STATE_OFF);
            Serial.println("[app] Sleep sent");
        }
        else if (input.length() > 0)
        {
            int16_t p = constrain(input.toInt(), -1875, 1875);
            g_essPower = p;
            vebus.setESSPower(p);
            Serial.printf("[app] ESS → %d W\n", p);
        }
    }
}
