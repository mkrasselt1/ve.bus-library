# VEBus — ESP32 Arduino Library for Victron Multiplus

PlatformIO / Arduino library for communicating with a **Victron Multiplus** inverter/charger over **VE.Bus RS485** from an ESP32.

Tested on the **[LilyGo T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485)** board.

## Credits & Sources

This library is a clean-room refactor of the excellent work by **PepeTheFroggie**:

> **[Victron-VE.Bus---esp32](https://github.com/PepeTheFroggie/Victron-VE.Bus---esp32)**
> Full ESS firmware for ESP32 including WiFi dashboard, Shelly power meter
> integration, SoC tracking, and VE.Bus protocol implementation.

Protocol reference and additional features derived from:

> **[j9brown/victron-mk3](https://github.com/j9brown/victron-mk3)**
> Python Home Assistant component via MK3 USB — comprehensive VE.Bus
> protocol implementation including device state control, RAM/setting
> read/write, and firmware version queries.

> **[pv-baxi/esp32ess](https://github.com/pv-baxi/esp32ess)**
> ESP32 ESS controller with extensive protocol documentation.

> **[Victron MK2 Protocol v3.14](https://www.victronenergy.com/upload/documents/Technical-Information-Interfacing-with-VE-Bus-products-MK2-Protocol-3-14.pdf)**
> Official Victron protocol specification.

## Features

- Internal FreeRTOS task handles all RS485 RX/TX with correct sync timing
- Thread-safe command queue — call any command from any core
- **ESS power setpoint** with automatic acknowledgement tracking
- **Flexible RAM variable reading** — read any combination of up to 6 RAM variables per request
- **Setting read/write** — read and write all Multiplus configuration settings
- **Device state control** — query state, force absorption/float/equalise
- **Switch state control** — on, off, charger-only, inverter-only
- **Firmware version** query (auto-chained two-part request)
- **Setting & RAM variable info** — query scale, offset, default, min, max
- Decodes charger/inverter status, LED bitmask, DC current, temperature, AC input limits
- Auto-direction support for MAX13487E transceiver
- Robust frame receiver — line noise or truncated frames cannot overflow buffers
- Comprehensive constants for RAM IDs, setting IDs, device states, LED/switch bitmasks

## ESS Setpoint Semantics

The setpoint controls power exchange on the **AC-IN (grid) side**, not AC-OUT:

| Value | Effect |
|-------|--------|
| `+300` | Invert 300 W from battery toward grid (reduces import / feeds back) |
| `-300` | Charge battery with 300 W from grid |
| `0` | Standby — grid pass-through only |

### Virtual Setpoint Mode (battery-neutral UPS)

In normal ESS mode, setpoint `0` still cycles the battery to cover AC-OUT loads.
**Virtual setpoint mode** subtracts AC-OUT load from the setpoint automatically so
you can control only the *extra* battery cycling on top of passthrough:

```
effective_setpoint = virtual_setpoint − ac_out_load
```

| Virtual | Effect |
|---------|--------|
| `0` | **Battery neutral** — grid supplies AC-OUT loads, battery idle |
| `+300` | Battery discharges 300 W extra to grid (on top of AC-OUT passthrough) |
| `-300` | Battery charges 300 W extra from grid (on top of AC-OUT passthrough) |

Enable with `enableVirtualSetpointMode(true)` and feed the AC-OUT load to
`setACOutLoad()` from your read loop (`VEBUS_RAM_OUTPUT_POWER`, RAM id 16).
The library re-sends the effective setpoint whenever the load changes by more
than the deadband (default 10 W).

## Hardware — LilyGo T-CAN485

The T-CAN485 uses a **MAX13487E** RS485 transceiver with auto-direction
(no DE pin — the driver enables automatically when data is sent).

| Signal | GPIO | MAX13487E Pin |
|--------|------|---------------|
| RS485 RX | 21 | RO (pin 1) |
| RS485 TX | 22 | DI (pin 4) |
| /RE (receiver enable) | 17 | /RE (pin 2) — active-low, used as RTS |
| /SHDN (shutdown) | 19 | /SHDN (pin 3) — **must be driven HIGH!** |

> **Important:** GPIO 19 controls the transceiver shutdown pin. If left
> floating or driven LOW, the transceiver is completely off (no RX or TX).
> The example calls `digitalWrite(19, HIGH)` in `setup()` before
> `vebus.begin()`.

### Hardware modification required

The T-CAN485 has **100pF capacitors (C9, C11)** and a **common-mode choke
(L2)** on the RS485 A/B lines. These are designed for typical Modbus speeds
(9600–19200 baud) but can cause signal integrity issues at the VE.Bus baud
rate of **256000 baud**.

**Remove the following components** near the RS485 transceiver for reliable
operation at 256 kbaud:

- **C9** (100pF) — on the B line
- **C11** (100pF) — on the A line
- **L2** (SDCW3225S-2-102TF, common-mode choke) — between transceiver and connector

The 120Ω termination resistor (R6) and TVS protection diodes can stay.

### Wiring

Connect **3 wires** between the T-CAN485 screw terminals and the Multiplus
VE.Bus RJ45 connector:

| T-CAN485 | Multiplus VE.Bus RJ45 |
|----------|-----------------------|
| A | Pin 3 (Data+) |
| B | Pin 4 (Data−) |
| GND | Pin 5 (GND) |

> **RS485 requires a ground connection.** Without GND the common-mode voltage
> drifts and the receiver reads noise.

## Quick Start

```cpp
#include <VEBus.h>

VEBus vebus;

void setup() {
    pinMode(19, OUTPUT);
    digitalWrite(19, HIGH);      // enable MAX13487E (/SHDN HIGH)
    vebus.begin(21, 22, 17);    // RX, TX, /RE — LilyGo T-CAN485
}

void loop() {
    // Queue commands — the library sends them at the right time
    vebus.setESSPower(300);      // push 300 W from battery toward grid
    vebus.requestReadRAM();      // request battery voltage + AC power

    if (vebus.hasNewData()) {
        vebus.clearNewData();
        Serial.printf("Bat: %.2f V  AC: %d W\n",
                      vebus.getBatVolt(), vebus.getACPower());
    }
    delay(5000);
}
```

### Reading extended RAM variables

```cpp
// Read mains voltage, mains current, inverter voltage, output power
const uint8_t ids[] = {
    VEBUS_RAM_UMAINS_RMS, VEBUS_RAM_IMAINS_RMS,
    VEBUS_RAM_UINVERTER_RMS, VEBUS_RAM_OUTPUT_POWER
};
vebus.readRAMVars(ids, 4);

// Later, check for response:
if (vebus.hasRAMVarResponse()) {
    for (int i = 0; i < vebus.getRAMVarCount(); i++)
        Serial.printf("  RAM[%d] = %d\n", i, vebus.getRAMVarValue(i));
    vebus.clearRAMVarResponse();
}
```

### Reading and writing settings

```cpp
// Read absorption voltage (setting ID 2)
vebus.readSetting(VEBUS_SETTING_UBAT_ABSORPTION);

if (vebus.hasSettingResponse()) {
    // Raw value /100 = volts (e.g. 5680 = 56.80 V)
    Serial.printf("Absorption: %.2f V\n", vebus.getSettingValue() / 100.0f);
    vebus.clearSettingResponse();
}

// Write AC input current limit (setting ID 6)
vebus.writeSetting(VEBUS_SETTING_IMAINS_LIMIT, 160);  // 16.0 A
```

### Device state and switch control

```cpp
// Query device state
vebus.requestDeviceState();
if (vebus.hasDeviceStateResponse()) {
    Serial.printf("State: %d  Sub: %d\n",
                  vebus.getDeviceState(), vebus.getDeviceSubState());
    vebus.clearDeviceStateResponse();
}

// Force charge mode
vebus.forceDeviceState(VEBUS_FORCE_ABSORPTION);
vebus.forceDeviceState(VEBUS_FORCE_FLOAT);

// Switch modes
vebus.setSwitchState(VEBUS_SWITCH_STATE_ON);            // charger + inverter
vebus.setSwitchState(VEBUS_SWITCH_STATE_CHARGER_ONLY);  // charger only
vebus.setSwitchState(VEBUS_SWITCH_STATE_INVERTER_ONLY); // inverter only
vebus.setSwitchState(VEBUS_SWITCH_STATE_OFF);            // both off (sleep)
```

## Examples

### `basic_ess` — Serial console control

Full example with automatic no-sync recovery and interactive serial commands:

| Serial input | Action |
|-------------|--------|
| `<number>` | Set ESS power (-1875..1875 W) |
| `w` | Wakeup (switch ON) |
| `s` | Sleep (switch OFF) |
| `co` | Charger-only mode |
| `io` | Inverter-only mode |
| `v` | Request firmware version |
| `d` | Request device state |
| `r` | Read extended RAM (mains V/A, inverter V/A, output W, SoC) |
| `fa` | Force absorption |
| `ff` | Force float |
| `fe` | Force equalise |
| `rs <id>` | Read setting by ID |
| `ws <id> <val>` | Write setting |
| `ri <id>` | Query RAM variable info (scale/offset) |
| `si <id>` | Query setting info (scale/offset/default/min/max) |
| `h` | Show help |

### `mqtt_ha` — MQTT with Home Assistant auto-discovery

Publishes all Multiplus data as HA entities. RAM variables are read in two
batches per cycle (6 + 4 IDs), device state is polled once per cycle.

**First setup (WiFiManager):** on first boot (or when the saved WiFi is not
reachable) the device opens an AP named `VEBus-Setup`. The captive portal asks
for WiFi credentials, the MQTT host/port/user/pass/device-id/topic-prefix and
a web admin password. Values are persisted to NVS.

**Web UI:** after joining WiFi the device serves
`http://<device-ip>/` (also `http://<device-id>.local/` via mDNS):

| Page | Access | Content |
|------|--------|---------|
| `/` | public, read-only | live values, VE.Bus/MQTT/WiFi status, 1 h / 6 h / 24 h charts (power, battery voltage, SoC; 1-minute averages kept in RAM) |
| `/admin/` | HTTP basic auth, user `admin`, default password `vebus` | ESS setpoint, switch state, battery-neutral mode, charge buttons, MQTT settings, ESS fail-safe timeout, admin password, OTA firmware upload, reboot, WiFi reset |

Change the default password — the dashboard shows a warning until you do.
JSON endpoints: `/api/state`, `/api/history`.

**Robustness:**

- Task watchdog on `loop()` (30 s) — a stalled loop reboots the board.
- WiFi supervisor — reassociates every 30 s while offline, reboots after 10 min.
- WiFi modem sleep disabled (keeps the web UI responsive).
- MQTT: 5 s socket timeout, reconnect every 5 s, HA discovery sent one entity
  per loop pass and re-sent when HA restarts (`homeassistant/status`).
- One JSON state message per cycle (`<prefix>/state`) instead of ~30 single
  publishes — far less blocking on a slow WiFi link.
- Optional ESS fail-safe: if no new setpoint arrives within N seconds the
  setpoint returns to 0 W.
- The last reset reason (power-on, watchdog, brownout, …) is shown on the
  dashboard and in the serial log.

**MQTT topics:** state → `<prefix>/state` (JSON, keys as in the table below),
availability → `<prefix>/status`, commands → `<prefix>/<command>/set`.
Versions before 1.2 published one topic per value; HA picks up the new
state topic automatically through discovery (entity ids are unchanged).

**Sensors (23 + diagnostics: WiFi signal, uptime, free heap, firmware version):**

| Sensor | JSON key | Source | Unit |
|--------|----------|--------|------|
| Battery Voltage | `bat_volt` | `getBatVolt()` | V |
| AC Power | `ac_power` | `getACPower()` | W |
| DC Current | `dc_current` | `getDCCurrent()` | A |
| Temperature | `temp` | `getTemp()` | °C |
| Charger Status | `charger_status` | `getChargerStatus()` | — |
| ESS Power | `ess_power` | local setpoint (virtual setpoint when virtual mode on) | W |
| Effective ESS Power | `ess_power_eff` | what's actually sent to the inverter | W |
| Mains Voltage | `mains_voltage` | RAM ID 0 | V |
| Mains Current | `mains_current` | RAM ID 1 | A |
| Inverter Voltage | `inv_voltage` | RAM ID 2 | V |
| Inverter Current | `inv_current` | RAM ID 3 | A |
| Output Power | `output_power` | RAM ID 16 | W |
| Mains Power | `mains_power` | RAM ID 15 | W |
| Battery Current | `bat_current` | RAM ID 5 | A |
| State of Charge | `soc` | RAM ID 13 | % |
| Mains Frequency | `mains_freq` | RAM ID 8 | Hz |
| Inverter Frequency | `inv_freq` | RAM ID 7 | Hz |
| LED On / LED Blink | `led_on` / `led_blink` | `getLEDon()` / `getLEDblink()` | — |
| AC Input Min / Max / Actual | `ac_in_min` / `ac_in_max` / `ac_in_actual` | `getMin/Max/ActInputCurrentLimit()` | A |
| AC Input Config | `ac_in_config` | `getAcInputConfiguration()` | — |
| Device State | `device_state` | `requestDeviceState()` | — |
| Charge Sub-State | `charge_sub_state` | `getDeviceSubState()` | — |
| Checksum Faults | `checksum_faults` | `getChecksumFaults()` | — |

**Binary Sensors (2):**

| Sensor | JSON key | Source |
|--------|----------|--------|
| VE.Bus Sync | `sync` | `hasNoSync()` |
| DC Allows Inverting | `dc_allows_inv` | `dcLevelAllowsInverting()` |

**Controls (8):**

| Entity | Type | Details |
|--------|------|---------|
| ESS Power Setpoint | Number | -1875..1875 W |
| Switch State | Select | on / off / charger_only / inverter_only |
| Battery-Neutral UPS Mode | Switch | toggles virtual setpoint mode |
| Wakeup Multiplus | Button | — |
| Sleep Multiplus | Button | — |
| Force Absorption | Button | — |
| Force Float | Button | — |
| Force Equalise | Button | — |

**Diagnostics:** `firmware_version` (VE.Bus firmware, requested at MQTT connect),
`rssi` (dBm), `uptime` (s), `free_heap` (bytes).

Commands: publish to `<prefix>/<command>/set` — `ess_power` (W),
`switch_state` (`on`/`off`/`charger_only`/`inverter_only`), `virtual_mode`
(`ON`/`OFF`), `wakeup`, `sleep`, `force_absorption`, `force_float`,
`force_equalise` (any payload).

### `raw_test` — RS485 hardware test

Minimal hex dumper that bypasses the library — useful for verifying RS485 wiring.

### `esphome_vebus.yaml` — ESPHome external component

The repo also ships an ESPHome wrapper under `components/vebus/` so the same
library is usable from a YAML-only ESPHome config. Pull it in with:

```yaml
external_components:
  - source: github://mkrasselt1/ve.bus-library
    components: [vebus]
```

The wrapper exposes a `vebus` hub plus `sensor`, `binary_sensor`, `number`, and
`switch` platforms — see `examples/esphome_vebus.yaml` for the full config.

The library's RS485 task runs on its own FreeRTOS core (configure with
`core: 1`), so blocking serial I/O never touches ESPHome's main loop.

## PlatformIO

The `platformio.ini` at the repo root has environments for all examples:

```bash
pio run -e basic_ess    # Serial console ESS control
pio run -e mqtt_ha      # MQTT → Home Assistant
```

## API Reference

### Initialisation

```cpp
void begin(int rxPin, int txPin, int dePin, int core = 0);
```

Starts UART at 256000 baud and launches an internal FreeRTOS task on the
specified core (default: 0). The task runs a tight loop with no delay to
catch sync timing.

### ESS Power

```cpp
void setESSPower(int16_t watts);
```

Queue an ESS power setpoint. Resets the command queue first (only the
latest setpoint matters). Must be sent at least every 60 seconds or the
Multiplus enters passthrough mode.

#### Virtual setpoint mode

```cpp
void    enableVirtualSetpointMode(bool enable, int16_t deadbandWatts = 10);
bool    isVirtualSetpointMode() const;
void    setACOutLoad(int16_t watts);     // feed RAM var 16 (OUTPUT_POWER)
int16_t getVirtualSetpoint()   const;    // what you last passed to setESSPower
int16_t getACOutLoad()         const;    // last AC-out load you fed in
int16_t getEffectiveESSPower() const;    // last value sent to inverter
```

When enabled, `setESSPower(W)` is interpreted as the virtual setpoint and the
library sends `W − ac_out_load` to the inverter. Call `setACOutLoad()` whenever
you have a fresh reading of RAM variable 16. The library re-sends the effective
setpoint automatically whenever the load drift exceeds `deadbandWatts`.

### Legacy RAM Read

```cpp
void requestReadRAM();           // request battery voltage + AC power
float   getBatVolt();            // battery voltage [V]
int16_t getACPower();            // AC power [W], positive = inverting
bool    hasNewData();            // response arrived
void    clearNewData();
```

### Flexible RAM Variable Read

```cpp
void readRAMVars(const uint8_t *ids, uint8_t count);  // up to 6 IDs
bool    hasRAMVarResponse();
void    clearRAMVarResponse();
uint8_t getRAMVarCount();
int16_t getRAMVarValue(uint8_t index);  // 0..count-1
```

### RAM Variable Write

```cpp
void writeRAMVar(uint8_t id, uint16_t value);
```

Writes to RAM only (no EEPROM). The acknowledgement is signalled via `isAcked()`.

### Setting Read

```cpp
void readSetting(uint8_t id);
bool     hasSettingResponse();
void     clearSettingResponse();
uint8_t  getSettingId();
uint16_t getSettingValue();
```

### Setting Write

```cpp
void writeSetting(uint8_t id, uint16_t value);
bool isSettingWriteAcked();
void clearSettingWriteAcked();
```

Writes to both RAM and EEPROM (persistent across power cycles).

### Switch State

```cpp
void setSwitchState(VEBusSwitchState state);
void requestSleep();   // shorthand for VEBUS_SWITCH_STATE_OFF
void requestWakeup();  // shorthand for VEBUS_SWITCH_STATE_ON
```

### Device State

```cpp
void requestDeviceState();
void forceDeviceState(VEBusForceState action);
bool    hasDeviceStateResponse();
void    clearDeviceStateResponse();
uint8_t getDeviceState();     // see VEBusDeviceState enum
uint8_t getDeviceSubState();  // see VEBusChargeSubState enum (when state=Charge)
```

### Firmware Version

```cpp
void requestVersion();         // auto-chains part 0 → part 1
bool     hasVersionResponse();
void     clearVersionResponse();
uint16_t getVersionLow();
uint16_t getVersionHigh();
```

### Setting & RAM Variable Info

```cpp
void requestSettingInfo(uint8_t id);
bool hasSettingInfoResponse();
void clearSettingInfoResponse();
const VEBusSettingInfo& getSettingInfo();
// VEBusSettingInfo: { id, scale, offset, defaultValue, minimum, maximum }

void requestRAMVarInfo(uint8_t id);
bool    hasRAMVarInfoResponse();
void    clearRAMVarInfoResponse();
uint8_t getRAMVarInfoId();
int16_t getRAMVarInfoScale();
int16_t getRAMVarInfoOffset();
```

### Broadcast Data (decoded from periodic Multiplus frames)

These values are updated continuously without sending any commands:

```cpp
// Frame 0x80 — Charger/Inverter condition
float getDCCurrent();              // battery current [A]
float getTemp();                   // device temperature [°C]
byte  getChargerStatus();          // operating mode byte
bool  dcLevelAllowsInverting();    // battery voltage sufficient?

// Frame 0x41 — MasterMultiLED
byte  getLEDon();                  // LED on bitmask (VEBUS_LED_*)
byte  getLEDblink();               // LED blink bitmask
byte  getLEDstatus();              // status register
byte  getAcInputConfiguration();   // AC input config byte
float getMinInputCurrentLimit();   // min AC input limit [A]
float getMaxInputCurrentLimit();   // max AC input limit [A]
float getActInputCurrentLimit();   // actual AC input limit [A]
byte  getSwitchRegister();         // switch register (VEBUS_SWITCH_*)
```

### Status

```cpp
bool     hasNoSync();          // no sync frame for > 1 s
bool     isAcked();            // last write command acknowledged
void     clearAcked();
uint32_t getChecksumFaults();  // RX checksum error counter
```

## Constants Reference

### RAM Variable IDs (`VEBUS_RAM_*`)

| ID | Constant | Description |
|----|----------|-------------|
| 0 | `VEBUS_RAM_UMAINS_RMS` | Mains voltage RMS (V) |
| 1 | `VEBUS_RAM_IMAINS_RMS` | Mains current RMS (A) |
| 2 | `VEBUS_RAM_UINVERTER_RMS` | Inverter output voltage RMS (V) |
| 3 | `VEBUS_RAM_IINVERTER_RMS` | Inverter output current RMS (A) |
| 4 | `VEBUS_RAM_UBAT` | Battery voltage (V) |
| 5 | `VEBUS_RAM_IBAT` | Battery current (A, signed) |
| 6 | `VEBUS_RAM_UBAT_RMS` | Battery ripple voltage |
| 7 | `VEBUS_RAM_INVERTER_PERIOD` | Inverter period (Hz = 10/value) |
| 8 | `VEBUS_RAM_MAINS_PERIOD` | Mains period (Hz = 10/value) |
| 9 | `VEBUS_RAM_SIGNED_AC_LOAD_CUR` | Signed AC load current |
| 10 | `VEBUS_RAM_VIRTUAL_SWITCH` | Virtual switch position |
| 11 | `VEBUS_RAM_IGNORE_AC_INPUT` | Ignore AC input state |
| 12 | `VEBUS_RAM_RELAY_STATE` | Multi-functional relay state |
| 13 | `VEBUS_RAM_CHARGE_STATE` | State of charge (~0.5% res.) |
| 14 | `VEBUS_RAM_INVERTER_POWER` | Inverter power, filtered (W) |
| 15 | `VEBUS_RAM_MAINS_POWER` | Mains power, filtered (W) |
| 16 | `VEBUS_RAM_OUTPUT_POWER` | Output power, filtered (W) |
| 17 | `VEBUS_RAM_INVERTER_POWER_UF` | Inverter power, unfiltered |
| 18 | `VEBUS_RAM_MAINS_POWER_UF` | Mains power, unfiltered |
| 19 | `VEBUS_RAM_OUTPUT_POWER_UF` | Output power, unfiltered |

> Use `requestRAMVarInfo(id)` to query the per-device scale and offset for
> converting raw values to physical units. Typical: voltage ×0.01, current ×0.1.

### Setting IDs (`VEBUS_SETTING_*`)

| ID | Constant | Description |
|----|----------|-------------|
| 0 | `VEBUS_SETTING_FLAGS0` | Primary flags (16-bit bitmask) |
| 1 | `VEBUS_SETTING_FLAGS1` | Secondary flags |
| 2 | `VEBUS_SETTING_UBAT_ABSORPTION` | Absorption voltage (raw/100 = V) |
| 3 | `VEBUS_SETTING_UBAT_FLOAT` | Float voltage (raw/100 = V) |
| 4 | `VEBUS_SETTING_IBAT_BULK` | Bulk charge current (A) |
| 5 | `VEBUS_SETTING_UINV_SETPOINT` | Inverter output voltage |
| 6 | `VEBUS_SETTING_IMAINS_LIMIT` | AC input 1 current limit |
| 7 | `VEBUS_SETTING_REPEATED_ABSORPTION_TIME` | Repeated absorption time |
| 8 | `VEBUS_SETTING_REPEATED_ABSORPTION_INTERVAL` | Repeated absorption interval |
| 9 | `VEBUS_SETTING_MAX_ABSORPTION_DURATION` | Max absorption duration |
| 10 | `VEBUS_SETTING_CHARGE_CHARACTERISTIC` | 0=variable, 1=fixed, 2=fixed+storage |
| 11 | `VEBUS_SETTING_UBAT_LOW_LIMIT` | Low battery cutoff (raw/100 = V) |
| 12 | `VEBUS_SETTING_UBAT_LOW_HYSTERESIS` | Low battery hysteresis |
| 13 | `VEBUS_SETTING_NUM_SLAVES` | Number of slaves connected |
| 14 | `VEBUS_SETTING_THREE_PHASE` | 0=3ph, 1=split 180°, 2=2-leg 120° |
| 44 | `VEBUS_SETTING_LOWEST_UMAINS` | Minimum acceptable mains voltage |
| 46 | `VEBUS_SETTING_HIGHEST_UMAINS` | Maximum acceptable mains voltage |
| 49 | `VEBUS_SETTING_IMAINS_LIMIT_AC2` | AC input 2 current limit |
| 60 | `VEBUS_SETTING_FLAGS2` | Additional flags |
| 61 | `VEBUS_SETTING_FLAGS3` | Additional flags |
| 64 | `VEBUS_SETTING_BATTERY_CAPACITY` | Battery capacity |
| 81 | `VEBUS_SETTING_GRID_CODE` | Grid code (0=none, 1=active) |

> Use `requestSettingInfo(id)` to query scale, offset, default, min, and max
> values for any setting.

### Device States (`VEBusDeviceState`)

| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `VEBUS_STATE_DOWN` | System down |
| 1 | `VEBUS_STATE_STARTUP` | Starting up |
| 2 | `VEBUS_STATE_OFF` | Switched off |
| 3 | `VEBUS_STATE_SLAVE` | Slave mode |
| 4 | `VEBUS_STATE_INVERT_FULL` | Inverting (full) |
| 5 | `VEBUS_STATE_INVERT_HALF` | Inverting (half) |
| 6 | `VEBUS_STATE_INVERT_AES` | Inverting (AES) |
| 7 | `VEBUS_STATE_POWER_ASSIST` | Power assist |
| 8 | `VEBUS_STATE_BYPASS` | Bypass / passthrough |
| 9 | `VEBUS_STATE_CHARGE` | Charging (see sub-states) |

### Charge Sub-States (`VEBusChargeSubState`)

| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `VEBUS_CHARGE_INIT` | Initialising |
| 1 | `VEBUS_CHARGE_BULK` | Bulk charge |
| 2 | `VEBUS_CHARGE_ABSORPTION` | Absorption charge |
| 3 | `VEBUS_CHARGE_FLOAT` | Float charge |
| 4 | `VEBUS_CHARGE_STORAGE` | Storage mode |
| 5 | `VEBUS_CHARGE_REPEATED_ABSORPTION` | Repeated absorption |
| 6 | `VEBUS_CHARGE_FORCED_ABSORPTION` | Forced absorption |
| 7 | `VEBUS_CHARGE_EQUALISE` | Equalisation |
| 8 | `VEBUS_CHARGE_BULK_STOPPED` | Bulk stopped |

### Switch States (`VEBusSwitchState`)

| Value | Constant | Description |
|-------|----------|-------------|
| 0x04 | `VEBUS_SWITCH_STATE_OFF` | Both charger and inverter off |
| 0x05 | `VEBUS_SWITCH_STATE_CHARGER_ONLY` | Charger on, inverter off |
| 0x06 | `VEBUS_SWITCH_STATE_INVERTER_ONLY` | Inverter on, charger off |
| 0x07 | `VEBUS_SWITCH_STATE_ON` | Both charger and inverter on |

### Force States (`VEBusForceState`)

| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `VEBUS_FORCE_INQUIRY` | Read state only |
| 1 | `VEBUS_FORCE_EQUALISE` | Force equalisation charge |
| 2 | `VEBUS_FORCE_ABSORPTION` | Force absorption charge |
| 3 | `VEBUS_FORCE_FLOAT` | Force float charge |

### LED Bitmask (`VEBUS_LED_*`)

| Bit | Constant | LED |
|-----|----------|-----|
| 0 | `VEBUS_LED_MAINS_ON` | Mains present |
| 1 | `VEBUS_LED_ABSORPTION` | Absorption charging |
| 2 | `VEBUS_LED_BULK` | Bulk charging |
| 3 | `VEBUS_LED_FLOAT` | Float charging |
| 4 | `VEBUS_LED_INVERTER_ON` | Inverter active |
| 5 | `VEBUS_LED_OVERLOAD` | Overload warning |
| 6 | `VEBUS_LED_LOW_BATTERY` | Low battery warning |
| 7 | `VEBUS_LED_TEMPERATURE` | Temperature warning |

### Switch Register Bitmask (`VEBUS_SWITCH_*`)

| Bit | Constant | Meaning |
|-----|----------|---------|
| 0 | `VEBUS_SWITCH_REMOTE_CHARGE` | Remote interface charger state |
| 1 | `VEBUS_SWITCH_REMOTE_INVERT` | Remote interface inverter state |
| 2 | `VEBUS_SWITCH_FRONT_UP` | Front panel switch UP |
| 3 | `VEBUS_SWITCH_FRONT_DOWN` | Front panel switch DOWN |
| 4 | `VEBUS_SWITCH_CHARGE` | Active: charger enabled |
| 5 | `VEBUS_SWITCH_INVERT` | Active: inverter enabled |
| 6 | `VEBUS_SWITCH_ONBOARD_REMOTE_INV` | Onboard remote inverter switch |
| 7 | `VEBUS_SWITCH_REMOTE_GENERATOR` | Remote generator selected |

## License

MIT — see [LICENSE](LICENSE).

Original firmware by PepeTheFroggie, MIT licensed.
