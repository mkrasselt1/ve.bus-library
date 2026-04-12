/**
 * mqtt_ha — VEBus → MQTT → Home Assistant auto-discovery
 *
 * Publishes all Multiplus data as HA sensors and exposes
 * ESS power (number), sleep and wakeup (buttons) as controls.
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

void publishDiscovery()
{
    // Sensors
    publishSensorConfig("Battery Voltage",  "vebus_bat_volt",     "V",  "voltage",     "bat_volt");
    publishSensorConfig("AC Power",         "vebus_ac_power",     "W",  "power",       "ac_power");
    publishSensorConfig("DC Current",       "vebus_dc_current",   "A",  "current",     "dc_current");
    publishSensorConfig("Temperature",      "vebus_temp",         "\xC2\xB0""C", "temperature", "temp");
    publishSensorConfig("LED On",           "vebus_led_on",       "",   "",            "led_on");
    publishSensorConfig("LED Blink",        "vebus_led_blink",    "",   "",            "led_blink");
    publishSensorConfig("Charger Status",   "vebus_charger",      "",   "",            "charger_status");
    publishSensorConfig("AC Input Min",     "vebus_ac_in_min",    "A",  "current",     "ac_in_min");
    publishSensorConfig("AC Input Max",     "vebus_ac_in_max",    "A",  "current",     "ac_in_max");
    publishSensorConfig("AC Input Actual",  "vebus_ac_in_actual", "A",  "current",     "ac_in_actual");
    publishSensorConfig("ESS Power",        "vebus_ess_state",    "W",  "power",       "ess_power");

    // Binary sensor
    publishBinarySensorConfig("VE.Bus Sync", "vebus_sync", "connectivity", "sync");

    // Number input
    publishNumberConfig();

    // Buttons
    publishButtonConfig("Wakeup Multiplus", "vebus_wakeup", "wakeup");
    publishButtonConfig("Sleep Multiplus",  "vebus_sleep",  "sleep");
}

// =======================================================================
// MQTT callback — handle commands from HA
// =======================================================================
void mqttCallback(char *topicStr, byte *payload, unsigned int length)
{
    // Null-terminate payload
    char msg[32];
    int len = min((unsigned int)31, length);
    memcpy(msg, payload, len);
    msg[len] = '\0';

    if (strstr(topicStr, "ess_power/set"))
    {
        int16_t watts = (int16_t)constrain(atoi(msg), -1875, 1875);
        g_essPower = watts;
        vebus.setESSPower(watts);
        // Publish state back for immediate HA feedback
        snprintf(payloadBuf, sizeof(payloadBuf), "%d", watts);
        mqtt.publish(topic("ess_power"), payloadBuf);
        Serial.printf("[MQTT] ESS → %d W\n", watts);
    }
    else if (strstr(topicStr, "sleep/set"))
    {
        vebus.requestSleep();
        Serial.println("[MQTT] Sleep sent");
    }
    else if (strstr(topicStr, "wakeup/set"))
    {
        vebus.requestWakeup();
        Serial.println("[MQTT] Wakeup sent");
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

        // Subscribe to all command topics
        char subTopic[128];
        snprintf(subTopic, sizeof(subTopic), "%s/+/set", TOPIC_PREFIX);
        mqtt.subscribe(subTopic);

        publishDiscovery();
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
    char val[16];

    dtostrf(vebus.getBatVolt(), 1, 2, val);
    mqtt.publish(topic("bat_volt"), val);

    snprintf(val, sizeof(val), "%d", vebus.getACPower());
    mqtt.publish(topic("ac_power"), val);

    dtostrf(vebus.getDCCurrent(), 1, 1, val);
    mqtt.publish(topic("dc_current"), val);

    dtostrf(vebus.getTemp(), 1, 1, val);
    mqtt.publish(topic("temp"), val);

    snprintf(val, sizeof(val), "%d", vebus.getLEDon());
    mqtt.publish(topic("led_on"), val);

    snprintf(val, sizeof(val), "%d", vebus.getLEDblink());
    mqtt.publish(topic("led_blink"), val);

    snprintf(val, sizeof(val), "%d", vebus.getChargerStatus());
    mqtt.publish(topic("charger_status"), val);

    dtostrf(vebus.getMinInputCurrentLimit(), 1, 1, val);
    mqtt.publish(topic("ac_in_min"), val);

    dtostrf(vebus.getMaxInputCurrentLimit(), 1, 1, val);
    mqtt.publish(topic("ac_in_max"), val);

    dtostrf(vebus.getActInputCurrentLimit(), 1, 1, val);
    mqtt.publish(topic("ac_in_actual"), val);

    snprintf(val, sizeof(val), "%d", (int)g_essPower);
    mqtt.publish(topic("ess_power"), val);

    mqtt.publish(topic("sync"), vebus.hasNoSync() ? "OFF" : "ON");
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
    static bool          ramRequested  = false;

    unsigned long now = millis();

    // MQTT keepalive + reconnect
    if (!mqtt.connected()) connectMqtt();
    mqtt.loop();

    // Queue ESS power every ESS_INTERVAL_MS
    if (now - lastESSMs >= ESS_INTERVAL_MS)
    {
        lastESSMs = now;
        vebus.setESSPower(g_essPower);
        ramRequested = false;
    }

    // Queue read RAM 500ms before next ESS
    if (!ramRequested && (now - lastESSMs >= (ESS_INTERVAL_MS - RAM_OFFSET_MS)))
    {
        vebus.requestReadRAM();
        ramRequested = true;
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

    // Publish sensor values
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
            int16_t p = constrain(input.toInt(), -1875, 1875);
            g_essPower = p;
            vebus.setESSPower(p);
            Serial.printf("[app] ESS → %d W\n", p);
        }
    }
}
