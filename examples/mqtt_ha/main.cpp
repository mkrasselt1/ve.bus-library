/**
 * mqtt_ha — VEBus → MQTT → Home Assistant auto-discovery + web dashboard
 *
 * - Publishes all Multiplus data as one JSON state message; HA entities are
 *   created via MQTT discovery (sensors, ESS number, switch-state select,
 *   battery-neutral switch, buttons, diagnostics).
 * - Built-in web UI on http://<device-ip>/ (or http://<hostname>.local/):
 *     /        live dashboard + 24 h history charts (public, read-only)
 *     /admin/  controls, settings, firmware update (HTTP basic auth,
 *              user "admin", default password "vebus" — change it!)
 * - Self-healing: task watchdog on the main loop, WiFi supervisor, MQTT
 *   reconnect with short timeouts, optional ESS setpoint fail-safe.
 *
 * Hardware: LilyGo T-CAN485 (ESP32, MAX13487E RS485 transceiver)
 *
 * Source:  https://github.com/mkrasselt1/ve.bus-library
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include <stdarg.h>
#include <VEBus.h>
#include "web_ui.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// =======================================================================
// Defaults — used the first time the device boots (no NVS entry yet) and
// pre-fill the captive portal form. Afterwards everything is editable at
// http://<device-ip>/admin/.
// =======================================================================
#define DEFAULT_MQTT_HOST     "192.0.2.100"
#define DEFAULT_MQTT_PORT     1883
#define DEFAULT_MQTT_USER     ""
#define DEFAULT_MQTT_PASS     ""
#define DEFAULT_DEVICE_ID     "vebus_multiplus"
#define DEFAULT_TOPIC_PREFIX  "vebus/multiplus"
#define DEFAULT_ADMIN_PASS    "vebus"
#define ADMIN_USER            "admin"
#define AP_SSID               "VEBus-Setup"
#define AP_PASS               ""    // open AP

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

// Robustness
#define WDT_TIMEOUT_S          30        // main loop must check in this often
#define WIFI_KICK_MS        30000UL      // retry WiFi association every 30 s
#define WIFI_REBOOT_MS     600000UL      // reboot after 10 min without WiFi

// History (for the dashboard charts): one averaged sample per minute, 24 h
#define HIST_INTERVAL_MS    60000UL
#define HIST_LEN             1440
#define HIST_GAP            INT16_MIN

// =======================================================================
// Globals
// =======================================================================
struct Config {
    char     mqttHost[64];
    uint16_t mqttPort;
    char     mqttUser[32];
    char     mqttPass[64];
    char     deviceId[32];
    char     prefix[64];
    char     adminPass[32];
    uint16_t essTimeoutS;       // 0 = off; else setpoint falls back to 0 W
} cfg;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
VEBus        vebus;
WiFiManager  wm;
Preferences  prefs;
WebServer    server(80);

char hostName[32];              // deviceId sanitised for DHCP / mDNS

// Live values from the extended RAM reads (raw, as reported by the device)
struct {
    int16_t mainsV, mainsA, invV, invA, outW, mainsW;
    int16_t batA, soc, mainsPeriod, invPeriod;
} live;

int16_t  g_essPower       = 0;
uint32_t g_lastEssCmdMs   = 0;
int      g_devState       = -1;
int      g_devSubState    = -1;
char     g_fwVersion[16]  = "";
uint32_t g_mqttReconnects = 0;
uint32_t g_wifiReconnects = 0;
bool     g_publishNow     = false;
uint8_t  g_discIdx        = 0xFF;    // next discovery entity (0xFF = done)
uint32_t g_restartAtMs    = 0;
bool     g_otaAuthed      = false;

char     payloadBuf[1536];

// History ring buffer
struct HistSample { int16_t out, mains, ess, batv, soc, bata; };
HistSample hist[HIST_LEN];
uint16_t   histHead  = 0;   // next write index
uint16_t   histCount = 0;
uint32_t   histLastMs = 0;
int32_t    histAcc[6] = {0};
uint16_t   histAccN   = 0;

// =======================================================================
// Small helpers
// =======================================================================

// Bounded JSON/text builder — never writes past the buffer.
struct Buf {
    char  *b;
    size_t cap, len;
    Buf(char *buf, size_t n) : b(buf), cap(n), len(0) { b[0] = '\0'; }
    void add(const char *fmt, ...)
    {
        if (len >= cap - 1) return;
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(b + len, cap - len, fmt, ap);
        va_end(ap);
        if (n > 0) len = min(cap - 1, len + (size_t)n);
    }
    void str(const char *s)     // JSON-escaped string incl. quotes
    {
        add("\"");
        for (; *s && len < cap - 2; s++) {
            if (*s == '"' || *s == '\\') add("\\%c", *s);
            else if ((uint8_t)*s < 0x20) add(" ");
            else { b[len++] = *s; b[len] = '\0'; }
        }
        add("\"");
    }
};

static const char *deviceStateName(int state)
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

static const char *chargeSubStateName(int sub)
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

static const char *resetReasonName()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic/exception";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "other watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP:return "deep sleep";
    case ESP_RST_EXT:      return "external pin";
    default:               return "unknown";
    }
}

static const char *switchStateName()
{
    byte sw = vebus.getSwitchRegister();
    bool chg = sw & VEBUS_SWITCH_CHARGE;
    bool inv = sw & VEBUS_SWITCH_INVERT;
    if (chg && inv) return "on";
    if (chg)        return "charger_only";
    if (inv)        return "inverter_only";
    return "off";
}

static float periodToHz(int16_t p) { return p > 0 ? 10.0f / (float)p : 0.0f; }

static bool isDefaultPassword() { return strcmp(cfg.adminPass, DEFAULT_ADMIN_PASS) == 0; }

// =======================================================================
// Config persistence (NVS via Preferences)
// =======================================================================
static void loadConfig()
{
    prefs.begin("vebus_mqtt", true);
    strlcpy(cfg.mqttHost,  prefs.getString("host",   DEFAULT_MQTT_HOST).c_str(),    sizeof(cfg.mqttHost));
    cfg.mqttPort =         prefs.getUShort("port",   DEFAULT_MQTT_PORT);
    strlcpy(cfg.mqttUser,  prefs.getString("user",   DEFAULT_MQTT_USER).c_str(),    sizeof(cfg.mqttUser));
    strlcpy(cfg.mqttPass,  prefs.getString("pass",   DEFAULT_MQTT_PASS).c_str(),    sizeof(cfg.mqttPass));
    strlcpy(cfg.deviceId,  prefs.getString("device", DEFAULT_DEVICE_ID).c_str(),    sizeof(cfg.deviceId));
    strlcpy(cfg.prefix,    prefs.getString("prefix", DEFAULT_TOPIC_PREFIX).c_str(), sizeof(cfg.prefix));
    strlcpy(cfg.adminPass, prefs.getString("admin",  DEFAULT_ADMIN_PASS).c_str(),   sizeof(cfg.adminPass));
    cfg.essTimeoutS =      prefs.getUShort("esstmo", 0);
    prefs.end();
    if (cfg.mqttPort == 0) cfg.mqttPort = DEFAULT_MQTT_PORT;
    if (!cfg.adminPass[0]) strlcpy(cfg.adminPass, DEFAULT_ADMIN_PASS, sizeof(cfg.adminPass));
}

static void saveConfig()
{
    prefs.begin("vebus_mqtt", false);
    prefs.putString("host",   cfg.mqttHost);
    prefs.putUShort("port",   cfg.mqttPort);
    prefs.putString("user",   cfg.mqttUser);
    prefs.putString("pass",   cfg.mqttPass);
    prefs.putString("device", cfg.deviceId);
    prefs.putString("prefix", cfg.prefix);
    prefs.putString("admin",  cfg.adminPass);
    prefs.putUShort("esstmo", cfg.essTimeoutS);
    prefs.end();
}

static void makeHostName()
{
    size_t j = 0;
    for (const char *s = cfg.deviceId; *s && j < sizeof(hostName) - 1; s++) {
        char c = *s;
        if (isalnum((unsigned char)c)) hostName[j++] = tolower(c);
        else if (j && hostName[j - 1] != '-') hostName[j++] = '-';
    }
    hostName[j] = '\0';
    if (!j) strlcpy(hostName, "vebus", sizeof(hostName));
}

// Drop the current MQTT session so the next connect uses the new settings.
static void applyMqttConfig()
{
    mqtt.disconnect();
    wifiClient.stop();
    mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
}

// =======================================================================
// Captive portal (first setup only)
// =======================================================================
WiFiManagerParameter *wmpHost, *wmpPort, *wmpUser, *wmpPass,
                     *wmpDeviceId, *wmpTopicPref, *wmpAdmin;

static void onPortalSave()
{
    strlcpy(cfg.mqttHost, wmpHost->getValue(),      sizeof(cfg.mqttHost));
    cfg.mqttPort = (uint16_t)atoi(wmpPort->getValue());
    if (cfg.mqttPort == 0) cfg.mqttPort = DEFAULT_MQTT_PORT;
    strlcpy(cfg.mqttUser, wmpUser->getValue(),      sizeof(cfg.mqttUser));
    strlcpy(cfg.mqttPass, wmpPass->getValue(),      sizeof(cfg.mqttPass));
    strlcpy(cfg.deviceId, wmpDeviceId->getValue(),  sizeof(cfg.deviceId));
    strlcpy(cfg.prefix,   wmpTopicPref->getValue(), sizeof(cfg.prefix));
    if (wmpAdmin->getValue()[0])
        strlcpy(cfg.adminPass, wmpAdmin->getValue(), sizeof(cfg.adminPass));
    saveConfig();
    Serial.println("[wm] config saved");
}

// =======================================================================
// Commands — shared by MQTT, web UI and serial console
// =======================================================================
static bool handleCommand(const char *cmd, const char *arg, const char *src)
{
    if (!strcmp(cmd, "ess_power")) {
        g_essPower = (int16_t)constrain(atoi(arg), -1875, 1875);
        g_lastEssCmdMs = millis();
        vebus.setESSPower(g_essPower);
        Serial.printf("[%s] ESS → %d W\n", src, g_essPower);
    }
    else if (!strcmp(cmd, "switch_state")) {
        if      (!strcmp(arg, "on"))            vebus.setSwitchState(VEBUS_SWITCH_STATE_ON);
        else if (!strcmp(arg, "off"))           vebus.setSwitchState(VEBUS_SWITCH_STATE_OFF);
        else if (!strcmp(arg, "charger_only"))  vebus.setSwitchState(VEBUS_SWITCH_STATE_CHARGER_ONLY);
        else if (!strcmp(arg, "inverter_only")) vebus.setSwitchState(VEBUS_SWITCH_STATE_INVERTER_ONLY);
        else return false;
        Serial.printf("[%s] Switch → %s\n", src, arg);
    }
    else if (!strcmp(cmd, "virtual_mode")) {
        bool on = (strcasecmp(arg, "ON") == 0 || atoi(arg) != 0);
        vebus.enableVirtualSetpointMode(on);
        Serial.printf("[%s] Virtual mode → %s\n", src, on ? "ON" : "OFF");
    }
    else if (!strcmp(cmd, "wakeup"))           vebus.setSwitchState(VEBUS_SWITCH_STATE_ON);
    else if (!strcmp(cmd, "sleep"))            vebus.setSwitchState(VEBUS_SWITCH_STATE_OFF);
    else if (!strcmp(cmd, "force_absorption")) vebus.forceDeviceState(VEBUS_FORCE_ABSORPTION);
    else if (!strcmp(cmd, "force_float"))      vebus.forceDeviceState(VEBUS_FORCE_FLOAT);
    else if (!strcmp(cmd, "force_equalise"))   vebus.forceDeviceState(VEBUS_FORCE_EQUALISE);
    else return false;

    if (strcmp(cmd, "ess_power") && strcmp(cmd, "switch_state") && strcmp(cmd, "virtual_mode"))
        Serial.printf("[%s] %s sent\n", src, cmd);
    g_publishNow = true;
    return true;
}

// =======================================================================
// State JSON — published to MQTT and served to the dashboard
// =======================================================================
static void buildStateJson(Buf &j)
{
    j.add("{\"bat_volt\":%.2f,\"ac_power\":%d,\"dc_current\":%.1f,\"temp\":%.1f,",
          vebus.getBatVolt(), (int)vebus.getACPower(), vebus.getDCCurrent(), vebus.getTemp());
    j.add("\"charger_status\":%u,\"ess_power\":%d,\"ess_power_eff\":%d,\"virtual_mode\":\"%s\",",
          (unsigned)vebus.getChargerStatus(), (int)g_essPower,
          (int)vebus.getEffectiveESSPower(), vebus.isVirtualSetpointMode() ? "ON" : "OFF");
    j.add("\"mains_voltage\":%d,\"mains_current\":%d,\"inv_voltage\":%d,\"inv_current\":%d,"
          "\"output_power\":%d,\"mains_power\":%d,",
          live.mainsV, live.mainsA, live.invV, live.invA, live.outW, live.mainsW);
    j.add("\"bat_current\":%d,\"soc\":%d,\"mains_freq\":%.1f,\"inv_freq\":%.1f,",
          live.batA, live.soc, periodToHz(live.mainsPeriod), periodToHz(live.invPeriod));
    j.add("\"led_on\":%u,\"led_blink\":%u,\"ac_in_min\":%.1f,\"ac_in_max\":%.1f,"
          "\"ac_in_actual\":%.1f,\"ac_in_config\":%u,",
          (unsigned)vebus.getLEDon(), (unsigned)vebus.getLEDblink(),
          vebus.getMinInputCurrentLimit(), vebus.getMaxInputCurrentLimit(),
          vebus.getActInputCurrentLimit(), (unsigned)vebus.getAcInputConfiguration());
    j.add("\"device_state\":\"%s\",\"charge_sub_state\":\"%s\",\"checksum_faults\":%lu,",
          g_devState < 0 ? "Unknown" : deviceStateName(g_devState),
          g_devState == VEBUS_STATE_CHARGE ? chargeSubStateName(g_devSubState) : "N/A",
          (unsigned long)vebus.getChecksumFaults());
    j.add("\"sync\":\"%s\",\"dc_allows_inv\":\"%s\",\"switch_state\":\"%s\",",
          vebus.hasNoSync() ? "OFF" : "ON",
          vebus.dcLevelAllowsInverting() ? "ON" : "OFF", switchStateName());
    j.add("\"firmware_version\":\"%s\",\"rssi\":%d,\"uptime\":%lu,\"free_heap\":%u}",
          g_fwVersion, (int)WiFi.RSSI(), (unsigned long)(millis() / 1000),
          (unsigned)ESP.getFreeHeap());
}

static void publishState()
{
    Buf j(payloadBuf, sizeof(payloadBuf));
    buildStateJson(j);
    char t[96];
    snprintf(t, sizeof(t), "%s/state", cfg.prefix);
    mqtt.publish(t, payloadBuf);
}

// =======================================================================
// HA MQTT auto-discovery — one entity per loop() pass so a slow broker
// never blocks the web UI. Unique ids match earlier versions of this
// example so existing HA entities are kept.
// =======================================================================
struct Entity {
    const char *comp;     // HA component
    const char *uid;      // unique id (prefixed with device id if devUid)
    bool        devUid;
    const char *name;
    const char *key;      // key in the state JSON (nullptr = stateless)
    const char *cmd;      // command suffix (nullptr = read-only)
    const char *extra;    // extra discovery JSON fields
};

#define MEAS  ",\"stat_cla\":\"measurement\""
#define DIAG  "\"ent_cat\":\"diagnostic\""

static const Entity ENTITIES[] = {
    // Core values
    {"sensor", "vebus_bat_volt",   false, "Battery Voltage",     "bat_volt",       nullptr, "\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\"" MEAS},
    {"sensor", "vebus_ac_power",   false, "AC Power",            "ac_power",       nullptr, "\"unit_of_meas\":\"W\",\"dev_cla\":\"power\"" MEAS},
    {"sensor", "vebus_dc_current", false, "DC Current",          "dc_current",     nullptr, "\"unit_of_meas\":\"A\",\"dev_cla\":\"current\"" MEAS},
    {"sensor", "vebus_temp",       false, "Temperature",         "temp",           nullptr, "\"unit_of_meas\":\"°C\",\"dev_cla\":\"temperature\"" MEAS},
    {"sensor", "vebus_charger",    false, "Charger Status",      "charger_status", nullptr, ""},
    {"sensor", "vebus_ess_state",  false, "ESS Power",           "ess_power",      nullptr, "\"unit_of_meas\":\"W\",\"dev_cla\":\"power\"" MEAS},
    {"sensor", "vebus_ess_eff",    false, "Effective ESS Power", "ess_power_eff",  nullptr, "\"unit_of_meas\":\"W\",\"dev_cla\":\"power\"" MEAS},
    // Extended RAM batch 1
    {"sensor", "vebus_mains_v",    false, "Mains Voltage",       "mains_voltage",  nullptr, "\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\"" MEAS},
    {"sensor", "vebus_mains_a",    false, "Mains Current",       "mains_current",  nullptr, "\"unit_of_meas\":\"A\",\"dev_cla\":\"current\"" MEAS},
    {"sensor", "vebus_inv_v",      false, "Inverter Voltage",    "inv_voltage",    nullptr, "\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\"" MEAS},
    {"sensor", "vebus_inv_a",      false, "Inverter Current",    "inv_current",    nullptr, "\"unit_of_meas\":\"A\",\"dev_cla\":\"current\"" MEAS},
    {"sensor", "vebus_output_w",   false, "Output Power",        "output_power",   nullptr, "\"unit_of_meas\":\"W\",\"dev_cla\":\"power\"" MEAS},
    {"sensor", "vebus_mains_w",    false, "Mains Power",         "mains_power",    nullptr, "\"unit_of_meas\":\"W\",\"dev_cla\":\"power\"" MEAS},
    // Extended RAM batch 2
    {"sensor", "vebus_bat_a",      false, "Battery Current",     "bat_current",    nullptr, "\"unit_of_meas\":\"A\",\"dev_cla\":\"current\"" MEAS},
    {"sensor", "vebus_soc",        false, "State of Charge",     "soc",            nullptr, "\"unit_of_meas\":\"%\",\"dev_cla\":\"battery\"" MEAS},
    {"sensor", "vebus_mains_hz",   false, "Mains Frequency",     "mains_freq",     nullptr, "\"unit_of_meas\":\"Hz\",\"dev_cla\":\"frequency\"" MEAS},
    {"sensor", "vebus_inv_hz",     false, "Inverter Frequency",  "inv_freq",       nullptr, "\"unit_of_meas\":\"Hz\",\"dev_cla\":\"frequency\"" MEAS},
    // LED, limits, status
    {"sensor", "vebus_led_on",       false, "LED On",            "led_on",           nullptr, ""},
    {"sensor", "vebus_led_blink",    false, "LED Blink",         "led_blink",        nullptr, ""},
    {"sensor", "vebus_ac_in_min",    false, "AC Input Min",      "ac_in_min",        nullptr, "\"unit_of_meas\":\"A\",\"dev_cla\":\"current\""},
    {"sensor", "vebus_ac_in_max",    false, "AC Input Max",      "ac_in_max",        nullptr, "\"unit_of_meas\":\"A\",\"dev_cla\":\"current\""},
    {"sensor", "vebus_ac_in_actual", false, "AC Input Actual",   "ac_in_actual",     nullptr, "\"unit_of_meas\":\"A\",\"dev_cla\":\"current\""},
    {"sensor", "vebus_ac_in_cfg",    false, "AC Input Config",   "ac_in_config",     nullptr, ""},
    {"sensor", "vebus_dev_state",    false, "Device State",      "device_state",     nullptr, ""},
    {"sensor", "vebus_charge_sub",   false, "Charge Sub-State",  "charge_sub_state", nullptr, ""},
    // Diagnostics
    {"sensor", "vebus_chksum",     false, "Checksum Faults",   "checksum_faults",  nullptr, DIAG},
    {"sensor", "vebus_fw",         false, "Firmware Version",  "firmware_version", nullptr, DIAG},
    {"sensor", "vebus_rssi",       false, "WiFi Signal",       "rssi",             nullptr, DIAG ",\"unit_of_meas\":\"dBm\",\"dev_cla\":\"signal_strength\"" MEAS},
    {"sensor", "vebus_uptime",     false, "Uptime",            "uptime",           nullptr, DIAG ",\"unit_of_meas\":\"s\",\"dev_cla\":\"duration\""},
    {"sensor", "vebus_heap",       false, "Free Heap",         "free_heap",        nullptr, DIAG ",\"unit_of_meas\":\"B\"" MEAS},
    // Binary sensors
    {"binary_sensor", "vebus_sync",  false, "VE.Bus Sync",         "sync",          nullptr, "\"dev_cla\":\"connectivity\""},
    {"binary_sensor", "vebus_dc_ok", false, "DC Allows Inverting", "dc_allows_inv", nullptr, "\"dev_cla\":\"power\""},
    // Controls
    {"number", "ess",              true,  "ESS Power Setpoint", "ess_power",    "ess_power",
        "\"min\":-1875,\"max\":1875,\"step\":1,\"unit_of_meas\":\"W\",\"mode\":\"box\""},
    {"select", "switch",           true,  "Switch State",       "switch_state", "switch_state",
        "\"options\":[\"on\",\"off\",\"charger_only\",\"inverter_only\"]"},
    {"switch", "vebus_virtual_mode", false, "Battery-Neutral UPS Mode", "virtual_mode", "virtual_mode", ""},
    {"button", "vebus_wakeup",      false, "Wakeup Multiplus", nullptr, "wakeup",           ""},
    {"button", "vebus_sleep",       false, "Sleep Multiplus",  nullptr, "sleep",            ""},
    {"button", "vebus_force_abs",   false, "Force Absorption", nullptr, "force_absorption", ""},
    {"button", "vebus_force_float", false, "Force Float",      nullptr, "force_float",      ""},
    {"button", "vebus_force_eq",    false, "Force Equalise",   nullptr, "force_equalise",   ""},
};
static const uint8_t ENTITY_COUNT = sizeof(ENTITIES) / sizeof(ENTITIES[0]);

static void publishEntity(const Entity &e)
{
    char uid[64], ct[128];
    if (e.devUid) snprintf(uid, sizeof(uid), "%s_%s", cfg.deviceId, e.uid);
    else          strlcpy(uid, e.uid, sizeof(uid));
    snprintf(ct, sizeof(ct), "homeassistant/%s/%s/config", e.comp, uid);

    Buf j(payloadBuf, sizeof(payloadBuf));
    j.add("{\"name\":\"%s\",\"uniq_id\":\"%s\",\"avty_t\":\"%s/status\"", e.name, uid, cfg.prefix);
    if (e.key)
        j.add(",\"stat_t\":\"%s/state\",\"val_tpl\":\"{{ value_json.%s }}\"", cfg.prefix, e.key);
    if (e.cmd)
        j.add(",\"cmd_t\":\"%s/%s/set\"", cfg.prefix, e.cmd);
    if (!strcmp(e.comp, "sensor") || !strcmp(e.comp, "binary_sensor"))
        j.add(",\"exp_aft\":60");
    if (e.extra[0])
        j.add(",%s", e.extra);
    j.add(",\"dev\":{\"ids\":[\"%s\"],\"name\":\"Victron Multiplus\",\"mfr\":\"Victron Energy\","
          "\"mdl\":\"Multiplus\",\"cu\":\"http://%s/\"}}",
          cfg.deviceId, WiFi.localIP().toString().c_str());
    mqtt.publish(ct, payloadBuf, true);
}

// =======================================================================
// MQTT
// =======================================================================
void mqttCallback(char *topicStr, byte *payload, unsigned int length)
{
    char msg[32];
    unsigned int len = min((unsigned int)sizeof(msg) - 1, length);
    memcpy(msg, payload, len);
    msg[len] = '\0';

    // HA restarted → resend discovery
    if (!strcmp(topicStr, "homeassistant/status")) {
        if (!strcmp(msg, "online")) g_discIdx = 0;
        return;
    }

    // <prefix>/<cmd>/set
    size_t plen = strlen(cfg.prefix);
    if (strncmp(topicStr, cfg.prefix, plen) || topicStr[plen] != '/') return;
    const char *rest = topicStr + plen + 1;
    const char *slash = strchr(rest, '/');
    if (!slash || strcmp(slash, "/set")) return;
    char cmd[32];
    strlcpy(cmd, rest, min(sizeof(cmd), (size_t)(slash - rest + 1)));
    handleCommand(cmd, msg, "MQTT");
}

static void connectMqtt()
{
    static uint32_t lastAttempt = 0;
    if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;
    if (lastAttempt && millis() - lastAttempt < MQTT_RECONNECT_MS) return;
    lastAttempt = millis();

    Serial.print("[MQTT] Connecting...");
    char statusTopic[96];
    snprintf(statusTopic, sizeof(statusTopic), "%s/status", cfg.prefix);

    bool ok = cfg.mqttUser[0]
        ? mqtt.connect(cfg.deviceId, cfg.mqttUser, cfg.mqttPass, statusTopic, 1, true, "offline")
        : mqtt.connect(cfg.deviceId, statusTopic, 1, true, "offline");

    if (ok) {
        Serial.println(" connected!");
        g_mqttReconnects++;
        mqtt.publish(statusTopic, "online", true);

        char sub[96];
        snprintf(sub, sizeof(sub), "%s/+/set", cfg.prefix);
        mqtt.subscribe(sub);
        mqtt.subscribe("homeassistant/status");

        g_discIdx = 0;          // discovery is sent incrementally from loop()
        g_publishNow = true;
        vebus.requestVersion();
        vebus.requestDeviceState();
    } else {
        Serial.printf(" failed (rc=%d)\n", mqtt.state());
    }
}

// =======================================================================
// History
// =======================================================================
static void histAccumulate()
{
    if (vebus.hasNoSync()) return;   // no data → leave a gap
    int32_t v[6] = { live.outW, live.mainsW, vebus.getEffectiveESSPower(),
                     (int32_t)lroundf(vebus.getBatVolt() * 100.0f), live.soc, live.batA };
    for (int i = 0; i < 6; i++) histAcc[i] += v[i];
    histAccN++;
}

static void histPush()
{
    HistSample s;
    int16_t *f = &s.out;
    for (int i = 0; i < 6; i++) {
        f[i] = histAccN ? (int16_t)constrain(histAcc[i] / histAccN, -32767, 32767) : HIST_GAP;
        histAcc[i] = 0;
    }
    histAccN = 0;
    hist[histHead] = s;
    histHead = (histHead + 1) % HIST_LEN;
    if (histCount < HIST_LEN) histCount++;
}

// =======================================================================
// Web server
// =======================================================================
static bool checkAuth()
{
    if (server.authenticate(ADMIN_USER, cfg.adminPass)) return true;
    server.requestAuthentication(BASIC_AUTH, "VEBus admin");
    return false;
}

static void sendJson(const char *json, int code = 200)
{
    server.sendHeader("Cache-Control", "no-store");
    server.send(code, "application/json", json);
}

static void handleState()
{
    Buf j(payloadBuf, sizeof(payloadBuf));
    buildStateJson(j);
    j.len--;                          // reopen the object
    j.add(",\"mqtt\":%s,\"mqtt_reconnects\":%lu,\"wifi_reconnects\":%lu,\"ip\":\"%s\","
          "\"host\":\"%s\",\"reset_reason\":\"%s\",\"default_pw\":%s,\"ess_timeout\":%u,"
          "\"device_id\":",
          mqtt.connected() ? "true" : "false", (unsigned long)g_mqttReconnects,
          (unsigned long)g_wifiReconnects, WiFi.localIP().toString().c_str(), hostName,
          resetReasonName(), isDefaultPassword() ? "true" : "false", cfg.essTimeoutS);
    j.str(cfg.deviceId);
    j.add("}");
    sendJson(payloadBuf);
}

// Streams ~40 kB without building it in RAM.
static void handleHistory()
{
    server.sendHeader("Cache-Control", "no-store");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");

    char buf[1200];
    Buf j(buf, sizeof(buf));
    j.add("{\"interval\":%lu,\"age\":%lu,\"cols\":[\"out\",\"mains\",\"ess\",\"batv\",\"soc\",\"bata\"],\"data\":[",
          HIST_INTERVAL_MS / 1000, (unsigned long)((millis() - histLastMs) / 1000));
    uint16_t start = (histHead + HIST_LEN - histCount) % HIST_LEN;
    for (uint16_t i = 0; i < histCount; i++) {
        const HistSample &s = hist[(start + i) % HIST_LEN];
        if (i) j.add(",");
        if (s.out == HIST_GAP) j.add("null");
        else j.add("[%d,%d,%d,%d,%d,%d]", s.out, s.mains, s.ess, s.batv, s.soc, s.bata);
        if (j.len > sizeof(buf) - 64) {
            server.sendContent(buf, j.len);
            j = Buf(buf, sizeof(buf));
        }
    }
    j.add("]}");
    server.sendContent(buf, j.len);
    server.sendContent("");
}

static void handleGetConfig()
{
    if (!checkAuth()) return;
    Buf j(payloadBuf, sizeof(payloadBuf));
    j.add("{\"host\":");   j.str(cfg.mqttHost);
    j.add(",\"port\":%u,\"user\":", cfg.mqttPort); j.str(cfg.mqttUser);
    j.add(",\"pass_set\":%s,\"device\":", cfg.mqttPass[0] ? "true" : "false"); j.str(cfg.deviceId);
    j.add(",\"prefix\":"); j.str(cfg.prefix);
    j.add(",\"ess_timeout\":%u,\"default_pw\":%s}", cfg.essTimeoutS,
          isDefaultPassword() ? "true" : "false");
    sendJson(payloadBuf);
}

static void handlePostConfig()
{
    if (!checkAuth()) return;
    if (server.hasArg("host") && server.arg("host").length())
        strlcpy(cfg.mqttHost, server.arg("host").c_str(), sizeof(cfg.mqttHost));
    if (server.hasArg("port")) {
        long p = server.arg("port").toInt();
        cfg.mqttPort = (p > 0 && p < 65536) ? (uint16_t)p : DEFAULT_MQTT_PORT;
    }
    if (server.hasArg("user"))
        strlcpy(cfg.mqttUser, server.arg("user").c_str(), sizeof(cfg.mqttUser));
    if (server.hasArg("pass") && server.arg("pass").length())          // empty = keep
        strlcpy(cfg.mqttPass, server.arg("pass").c_str(), sizeof(cfg.mqttPass));
    if (server.hasArg("clear_pass"))
        cfg.mqttPass[0] = '\0';
    if (server.hasArg("device") && server.arg("device").length())
        strlcpy(cfg.deviceId, server.arg("device").c_str(), sizeof(cfg.deviceId));
    if (server.hasArg("prefix") && server.arg("prefix").length())
        strlcpy(cfg.prefix, server.arg("prefix").c_str(), sizeof(cfg.prefix));
    if (server.hasArg("ess_timeout"))
        cfg.essTimeoutS = (uint16_t)constrain(server.arg("ess_timeout").toInt(), 0, 65535);
    if (server.hasArg("admin_pass")) {
        String p = server.arg("admin_pass");
        if (p.length() >= 4 && p.length() < sizeof(cfg.adminPass))
            strlcpy(cfg.adminPass, p.c_str(), sizeof(cfg.adminPass));
        else if (p.length()) {
            sendJson("{\"ok\":false,\"error\":\"Admin password must be 4-31 characters\"}", 400);
            return;
        }
    }
    saveConfig();
    makeHostName();
    applyMqttConfig();
    Serial.println("[web] config saved — reconnecting MQTT");
    sendJson("{\"ok\":true}");
}

static void handleControl()
{
    if (!checkAuth()) return;
    String cmd = server.arg("cmd"), val = server.arg("value");
    if (handleCommand(cmd.c_str(), val.c_str(), "web"))
        sendJson("{\"ok\":true}");
    else
        sendJson("{\"ok\":false,\"error\":\"unknown command\"}", 400);
}

static void handleReboot()
{
    if (!checkAuth()) return;
    sendJson("{\"ok\":true}");
    g_restartAtMs = millis() + 1000;
}

static void handleWifiReset()
{
    if (!checkAuth()) return;
    sendJson("{\"ok\":true}");
    server.client().flush();
    delay(200);
    wm.resetSettings();       // forget WiFi → captive portal on next boot
    ESP.restart();
}

static void handleUpdateDone()
{
    if (!checkAuth()) return;
    bool ok = g_otaAuthed && !Update.hasError();
    server.sendHeader("Connection", "close");
    server.send(ok ? 200 : 500, "text/plain", ok ? "OK — rebooting" : Update.errorString());
    if (ok) g_restartAtMs = millis() + 1000;
}

static void handleUpdateUpload()
{
    HTTPUpload &up = server.upload();
    esp_task_wdt_reset();                       // large uploads take a while
    if (up.status == UPLOAD_FILE_START) {
        g_otaAuthed = server.authenticate(ADMIN_USER, cfg.adminPass);
        if (!g_otaAuthed) return;
        Serial.printf("[ota] start: %s\n", up.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (!g_otaAuthed) {
        return;
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_END) {
        if (Update.end(true)) Serial.printf("[ota] done: %u bytes\n", up.totalSize);
        else                  Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        Serial.println("[ota] aborted");
    }
}

static void sendPage(const char *html)
{
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "text/html; charset=utf-8", html);
}

static void setupWebServer()
{
    server.on("/",                 HTTP_GET,  [] { sendPage(INDEX_HTML); });
    server.on("/style.css",        HTTP_GET,  [] {
        server.sendHeader("Cache-Control", "max-age=3600");
        server.send_P(200, "text/css", STYLE_CSS);
    });
    server.on("/api/state",        HTTP_GET,  handleState);
    server.on("/api/history",      HTTP_GET,  handleHistory);
    server.on("/admin",            HTTP_GET,  [] { server.sendHeader("Location", "/admin/"); server.send(302); });
    server.on("/admin/",           HTTP_GET,  [] { if (checkAuth()) sendPage(ADMIN_HTML); });
    server.on("/admin/api/config", HTTP_GET,  handleGetConfig);
    server.on("/admin/api/config", HTTP_POST, handlePostConfig);
    server.on("/admin/api/control",HTTP_POST, handleControl);
    server.on("/admin/api/reboot", HTTP_POST, handleReboot);
    server.on("/admin/api/wifireset", HTTP_POST, handleWifiReset);
    server.on("/admin/update",     HTTP_POST, handleUpdateDone, handleUpdateUpload);
    server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
    server.begin();
}

// =======================================================================
// Watchdogs
// =======================================================================
static void setupTaskWatchdog()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t c = {};
    c.timeout_ms     = WDT_TIMEOUT_S * 1000;
    c.idle_core_mask = 0;
    c.trigger_panic  = true;
    esp_task_wdt_reconfigure(&c);
#else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
    esp_task_wdt_add(NULL);   // loopTask: reboot if loop() stalls
}

// ESP32 auto-reconnect occasionally gives up (AP reboot, DHCP hiccup).
// Kick it periodically and reboot as a last resort.
static void superviseWiFi(uint32_t now)
{
    static bool     down = false;
    static uint32_t downSince = 0, lastKick = 0;

    if (WiFi.status() == WL_CONNECTED) {
        if (down) {
            Serial.printf("[wifi] reconnected (%s)\n", WiFi.localIP().toString().c_str());
            g_wifiReconnects++;
        }
        down = false;
        return;
    }
    if (!down) {
        down = true;
        downSince = lastKick = now;
        Serial.println("[wifi] connection lost");
    }
    if (now - lastKick >= WIFI_KICK_MS) {
        lastKick = now;
        Serial.println("[wifi] reconnecting...");
        WiFi.disconnect();
        WiFi.begin();
    }
    if (now - downSince >= WIFI_REBOOT_MS) {
        Serial.println("[wifi] offline too long — rebooting");
        ESP.restart();
    }
}

// =======================================================================
// Serial console: w=wake, s=sleep, number=ESS watts (non-blocking)
// =======================================================================
static void pollSerial()
{
    static char line[24];
    static uint8_t n = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && c != '\r') {
            if (n < sizeof(line) - 1) line[n++] = c;
            continue;
        }
        line[n] = '\0';
        n = 0;
        if (!line[0]) continue;
        if (!strcasecmp(line, "w"))      handleCommand("wakeup", "", "serial");
        else if (!strcasecmp(line, "s")) handleCommand("sleep", "", "serial");
        else                             handleCommand("ess_power", line, "serial");
    }
}

// =======================================================================
// setup()
// =======================================================================
void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== VEBus MQTT → Home Assistant ===");
    Serial.printf("Reset reason: %s\n", resetReasonName());

    // Enable RS485 transceiver
    pinMode(VEBUS_PIN_SHDN, OUTPUT);
    digitalWrite(VEBUS_PIN_SHDN, HIGH);

    // Start VEBus (internal task on core 0)
    vebus.begin(VEBUS_PIN_RX, VEBUS_PIN_TX, VEBUS_PIN_RE);
    Serial.println("VE.Bus started.");

    loadConfig();
    makeHostName();
    Serial.printf("[cfg] MQTT=%s:%u user='%s' device='%s' prefix='%s'\n",
                  cfg.mqttHost, cfg.mqttPort, cfg.mqttUser, cfg.deviceId, cfg.prefix);

    // Captive portal parameters (only shown when the portal opens)
    char portBuf[8];
    snprintf(portBuf, sizeof(portBuf), "%u", cfg.mqttPort);
    wmpHost      = new WiFiManagerParameter("host",   "MQTT host",         cfg.mqttHost, 63);
    wmpPort      = new WiFiManagerParameter("port",   "MQTT port",         portBuf,       5);
    wmpUser      = new WiFiManagerParameter("user",   "MQTT username",     cfg.mqttUser, 31);
    wmpPass      = new WiFiManagerParameter("pass",   "MQTT password",     cfg.mqttPass, 63);
    wmpDeviceId  = new WiFiManagerParameter("device", "HA device id",      cfg.deviceId, 31);
    wmpTopicPref = new WiFiManagerParameter("prefix", "MQTT topic prefix", cfg.prefix,   63);
    wmpAdmin     = new WiFiManagerParameter("admin",  "Web admin password (user: admin, default: vebus)", "", 31);
    wm.addParameter(wmpHost);
    wm.addParameter(wmpPort);
    wm.addParameter(wmpUser);
    wm.addParameter(wmpPass);
    wm.addParameter(wmpDeviceId);
    wm.addParameter(wmpTopicPref);
    wm.addParameter(wmpAdmin);
    wm.setSaveParamsCallback(onPortalSave);
    wm.setConfigPortalBlocking(true);
    wm.setConnectTimeout(30);
    wm.setConfigPortalTimeout(300);   // no one configured it → reboot and retry
    wm.setHostname(hostName);

    // First boot or no reachable WiFi → captive portal AP. Otherwise STA connect.
    if (!wm.autoConnect(AP_SSID, AP_PASS)) {
        Serial.println("[wm] autoConnect failed/timed out — rebooting");
        delay(1000);
        ESP.restart();
    }
    makeHostName();                   // device id may have changed in the portal
    WiFi.setSleep(false);             // modem sleep makes the web UI sluggish/unreachable
    WiFi.setAutoReconnect(true);
    Serial.printf("[wifi] connected (%s)\n", WiFi.localIP().toString().c_str());

    if (MDNS.begin(hostName)) MDNS.addService("http", "tcp", 80);
    setupWebServer();
    Serial.printf("[web] dashboard at http://%s/ (http://%s.local/)\n",
                  WiFi.localIP().toString().c_str(), hostName);
    if (isDefaultPassword())
        Serial.println("[web] WARNING: admin password is the default — change it at /admin/");

    // MQTT — short timeouts so a dead broker never stalls the loop for long
    mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
    mqtt.setBufferSize(sizeof(payloadBuf) + 128);
    mqtt.setSocketTimeout(5);
    mqtt.setKeepAlive(30);
    mqtt.setCallback(mqttCallback);

    histLastMs = millis();
    setupTaskWatchdog();
}

// =======================================================================
// loop()
// =======================================================================
void loop()
{
    static uint32_t lastESSMs     = 0;
    static uint32_t lastPublishMs = 0;
    static bool     ramRequested     = false;
    static bool     extRam1Requested = false;
    static bool     extRam2Requested = false;
    static bool     stateRequested   = false;

    esp_task_wdt_reset();
    uint32_t now = millis();

    superviseWiFi(now);
    server.handleClient();

    connectMqtt();
    mqtt.loop();

    // Discovery, one entity per pass
    if (g_discIdx < ENTITY_COUNT && mqtt.connected())
        publishEntity(ENTITIES[g_discIdx++]);

    // ESS fail-safe: no fresh setpoint for essTimeoutS → back to 0 W
    if (cfg.essTimeoutS && g_essPower != 0 &&
        now - g_lastEssCmdMs >= (uint32_t)cfg.essTimeoutS * 1000UL)
    {
        g_essPower = 0;
        vebus.setESSPower(0);
        g_publishNow = true;
        Serial.println("[app] ESS setpoint timed out — fallback to 0 W");
    }

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
        extRam2Requested = true;
    }

    // Queue device state request
    if (!stateRequested && (now - lastESSMs >= STATE_OFFSET_MS))
    {
        vebus.requestDeviceState();
        stateRequested = true;
    }

    // Extended RAM responses — matched by RAM id, so a late or dropped
    // response can never land in the wrong variable.
    if (vebus.hasRAMVarResponse())
    {
        for (uint8_t i = 0; i < vebus.getRAMVarCount(); i++)
        {
            int16_t v = vebus.getRAMVarValue(i);
            switch (vebus.getRAMVarId(i))
            {
            case VEBUS_RAM_UMAINS_RMS:      live.mainsV = v; break;
            case VEBUS_RAM_IMAINS_RMS:      live.mainsA = v; break;
            case VEBUS_RAM_UINVERTER_RMS:   live.invV   = v; break;
            case VEBUS_RAM_IINVERTER_RMS:   live.invA   = v; break;
            case VEBUS_RAM_OUTPUT_POWER:    live.outW   = v;
                                            vebus.setACOutLoad(v);   // virtual setpoint mode
                                            break;
            case VEBUS_RAM_MAINS_POWER:     live.mainsW = v; break;
            case VEBUS_RAM_IBAT:            live.batA   = v; break;
            case VEBUS_RAM_CHARGE_STATE:    live.soc    = v; break;
            case VEBUS_RAM_MAINS_PERIOD:    live.mainsPeriod = v; break;
            case VEBUS_RAM_INVERTER_PERIOD: live.invPeriod   = v; break;
            default: break;
            }
        }
        vebus.clearRAMVarResponse();
    }

    if (vebus.hasDeviceStateResponse())
    {
        g_devState    = vebus.getDeviceState();
        g_devSubState = vebus.getDeviceSubState();
        vebus.clearDeviceStateResponse();
    }

    if (vebus.hasVersionResponse())
    {
        snprintf(g_fwVersion, sizeof(g_fwVersion), "%u.%u",
                 vebus.getVersionHigh(), vebus.getVersionLow());
        vebus.clearVersionResponse();
    }

    // Auto-wakeup on no-sync
    if (vebus.hasNoSync())
    {
        static uint32_t lastWakeupMs = 0;
        if (now - lastWakeupMs >= WAKEUP_RETRY_MS)
        {
            lastWakeupMs = now;
            vebus.requestWakeup();
            Serial.println("[app] No sync — queued wakeup");
        }
    }

    // Publish state periodically (or right after a command)
    if (now - lastPublishMs >= PUBLISH_INTERVAL_MS || g_publishNow)
    {
        if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) histAccumulate();
        lastPublishMs = now;
        g_publishNow  = false;
        if (vebus.hasNewData()) vebus.clearNewData();
        if (mqtt.connected()) publishState();
    }

    if (now - histLastMs >= HIST_INTERVAL_MS)
    {
        histLastMs += HIST_INTERVAL_MS;
        histPush();
    }

    pollSerial();

    if (g_restartAtMs && (int32_t)(now - g_restartAtMs) >= 0)
    {
        Serial.println("[app] restarting");
        mqtt.disconnect();
        delay(100);
        ESP.restart();
    }
}
