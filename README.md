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

- Internal FreeRTOS task handles all RS485 RX/TX with correct sync timing
- Thread-safe command queue — call `setESSPower()` etc. from any core
- ESS power setpoint, sleep/wakeup commands, RAM variable reads
- Decodes charger/inverter status, LED bitmask, DC current, temperature, AC input limits
- Auto-direction support for MAX13487E transceiver (no DE pin needed)
- Automatic no-sync recovery with wakeup retry (in the example)

## ESS Setpoint Semantics

The setpoint controls power exchange on the **AC-IN (grid) side**, not AC-OUT:

| Value | Effect |
|-------|--------|
| `+300` | Invert 300 W from battery toward grid (reduces import / feeds back) |
| `-300` | Charge battery with 300 W from grid |
| `0` | Standby — grid pass-through only |

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

See [`examples/basic_ess/main.cpp`](examples/basic_ess/main.cpp) for a full
example with automatic no-sync recovery and serial console commands:

| Serial input | Action |
|-------------|--------|
| `w` | Wake up Multiplus |
| `s` | Sleep Multiplus |
| `300` | Set ESS to +300 W (invert) |
| `-500` | Set ESS to −500 W (charge) |
| `0` | Standby |

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
// Starts UART and launches internal task on the specified core (default: 0)
void begin(int rxPin, int txPin, int dePin, int core = 0);
```

### Thread-safe commands (call from any core/task)
```cpp
void setESSPower(int16_t watts);  // queue ESS power setpoint
void requestReadRAM();            // request battery voltage + AC power
void requestSleep();              // put Multiplus to sleep
void requestWakeup();             // wake Multiplus from sleep
```

### Status
```cpp
bool     hasNoSync();        // no sync frame for > 1 s
bool     hasNewData();       // RAM read response arrived
void     clearNewData();
bool     isAcked();          // last ESS command was acknowledged
void     clearAcked();
uint32_t getChecksumFaults();
```

### Decoded data (safe to read from any core)
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
byte    getSwitchRegister();
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
