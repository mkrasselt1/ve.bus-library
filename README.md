# VEBus — ESP32 Arduino Library for Victron Multiplus

PlatformIO / Arduino library for communicating with a **Victron Multiplus** inverter/charger over **VE.Bus RS485** from an ESP32.

Tested on the **[LilyGo T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485)** board.

## Credits & Sources

This library is a clean-room refactor of the excellent work by **PepeTheFroggie**:

> **[Victron-VE.Bus---esp32](https://github.com/PepeTheFroggie/Victron-VE.Bus---esp32)**
> Full ESS firmware for ESP32 including WiFi dashboard, Shelly power meter
> integration, SoC tracking, and VE.Bus protocol implementation.

The protocol framing, byte-stuffing rules, command structure, and frame
decoder in this library are all derived from that project. If you need a
complete ready-to-run ESS controller (rather than a reusable library),
start there.

## Features

- RS485 half-duplex UART init (ESP32 hardware RS485 mode)
- Sync frame detection with rolling frame-number tracking
- Send **ESS power setpoint** (`sendESSPower`)
- Send **sleep / wakeup** commands
- Request **battery voltage + AC power** via RAM read (`sendReadRAM`)
- Decode charger/inverter status, LED bitmask, DC current, temperature, AC input limits
- No internal RTOS task — call `handle()` from your own loop or task

## ESS Setpoint Semantics

The setpoint controls power exchange on the **AC-IN (grid) side**, not AC-OUT:

| Value | Effect |
|-------|--------|
| `+300` | Invert 300 W from battery toward grid (reduces import / feeds back) |
| `-300` | Charge battery with 300 W from grid |
| `0` | Standby — grid pass-through only |

## Hardware — LilyGo T-CAN485

| Signal | GPIO |
|--------|------|
| RS485 RX | 21 |
| RS485 TX | 22 |
| RS485 DE | 17 |

Wiring to Multiplus: connect **A (+)** and **B (−)** RS485 lines and share **GND**.

## Quick Start

```cpp
#include <VEBus.h>

VEBus vebus;

void setup() {
    vebus.begin(21, 22, 17);   // RX, TX, DE — LilyGo T-CAN485
}

void loop() {
    vebus.handle();

    if (vebus.isSynced() && millis() >= vebus.getSyncTime() + 8) {
        vebus.clearSync();
        vebus.sendESSPower(300);   // push 300 W from battery toward grid
    }

    if (vebus.hasNewData()) {
        vebus.clearNewData();
        Serial.printf("Bat: %.2f V  AC: %d W\n",
                      vebus.getBatVolt(), vebus.getACPower());
    }
}
```

See [`examples/basic_ess/main.cpp`](examples/basic_ess/main.cpp) for a full
dual-core FreeRTOS example with automatic no-sync recovery.

## PlatformIO

The `platformio.ini` at the repo root compiles the `basic_ess` example directly
— open the folder in VS Code with the PlatformIO extension and hit **Build**.

```ini
[env:lilygo-t-can485]
platform  = espressif32
board     = esp32dev
framework = arduino
src_dir   = examples/basic_ess
lib_extra_dirs = .
```

## API Reference

### Initialisation
```cpp
void begin(int rxPin, int txPin, int dePin, uint32_t baud = 256000);
```

### Main loop
```cpp
void handle();   // call as often as possible
```

### TX commands (call after isSynced())
```cpp
void sendESSPower(int16_t watts);
void sendReadRAM();
void sendSleep();
void sendWakeup();
```

### Status
```cpp
bool          isSynced();
unsigned long getSyncTime();
void          clearSync();
bool          hasNoSync();        // no sync frame for > 1 s
bool          hasNewData();       // RAM read response arrived
void          clearNewData();
bool          isAcked();
uint32_t      getChecksumFaults();
```

### Decoded data
```cpp
float   getBatVolt();             // battery voltage [V]
int16_t getACPower();             // AC power [W]  positive = inverting
float   getDCCurrent();           // DC battery current [A]
float   getTemp();                // Multiplus temperature [°C]
byte    getLEDon();               // LED bitmask — use VEBUS_LED_* constants
byte    getLEDblink();
byte    getLEDstatus();
byte    getChargerStatus();
bool    dcLevelAllowsInverting();
float   getMinInputCurrentLimit();
float   getMaxInputCurrentLimit();
float   getActInputCurrentLimit();
```

### LED constants
```cpp
VEBUS_LED_MAINS_ON
VEBUS_LED_ABSORPTION
VEBUS_LED_BULK
VEBUS_LED_FLOAT
VEBUS_LED_INVERTER_ON
VEBUS_LED_OVERLOAD
VEBUS_LED_LOW_BATTERY
VEBUS_LED_TEMPERATURE
```

## License

MIT — see [LICENSE](LICENSE).

Original firmware by PepeTheFroggie, MIT licensed.
