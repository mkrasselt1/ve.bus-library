#pragma once

#include <Arduino.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// -------------------------------------------------------------------
// LED bitmask constants (returned by getLEDon() / getLEDblink())
// -------------------------------------------------------------------
#define VEBUS_LED_MAINS_ON    (1 << 0)
#define VEBUS_LED_ABSORPTION  (1 << 1)
#define VEBUS_LED_BULK        (1 << 2)
#define VEBUS_LED_FLOAT       (1 << 3)
#define VEBUS_LED_INVERTER_ON (1 << 4)
#define VEBUS_LED_OVERLOAD    (1 << 5)
#define VEBUS_LED_LOW_BATTERY (1 << 6)
#define VEBUS_LED_TEMPERATURE (1 << 7)

// -------------------------------------------------------------------
// Command types for the internal queue
// -------------------------------------------------------------------
enum VEBusCmdType : uint8_t {
    VEBUS_CMD_ESS_POWER = 1,   // write ESS power setpoint
    VEBUS_CMD_READ_RAM  = 2,   // request bat voltage + AC power
    VEBUS_CMD_SLEEP     = 3,   // put Multiplus to sleep
    VEBUS_CMD_WAKEUP    = 4,   // wake Multiplus from sleep
};

struct VEBusCmd {
    VEBusCmdType type;
    int16_t      power;  // only used for VEBUS_CMD_ESS_POWER
};

// -------------------------------------------------------------------
// VEBus
//
// Runs its own FreeRTOS task in a tight loop (no delay) to handle
// RS485 RX/TX with correct sync timing — matching the original
// architecture from PepeTheFroggie's firmware.
//
// Usage:
//   VEBus vebus;
//   void setup() { vebus.begin(RX, TX, DE); }
//   void loop() {
//     vebus.setESSPower(watts);         // queued, sent on next sync
//     vebus.requestReadRAM();           // queued
//     Serial.printf("V=%.2f\n", vebus.getBatVolt());
//   }
// -------------------------------------------------------------------
class VEBus {
public:
    VEBus();

    // Start the RS485 driver and launch the internal task.
    // The task runs a tight loop on the specified core (default: core 1,
    // same as Arduino loop() in the original firmware).
    void begin(int rxPin, int txPin, int dePin, int core = 0);

    // ----------------------------------------------------------------
    // Thread-safe commands — queue a command for the next sync slot.
    // Can be called from any core/task at any time.
    // ----------------------------------------------------------------

    // Write ESS power setpoint (positive = invert, negative = charge, 0 = standby)
    void setESSPower(int16_t watts);

    // Request battery voltage + AC power (response arrives in getters)
    void requestReadRAM();

    // Sleep / wake the Multiplus
    void requestSleep();
    void requestWakeup();

    // ----------------------------------------------------------------
    // Decoded data — safe to read from any core (atomic on Xtensa)
    // ----------------------------------------------------------------
    float   getBatVolt()     const { return _BatVolt; }
    int16_t getACPower()     const { return _ACPower; }
    float   getDCCurrent()   const { return _multiplusDcCurrent; }
    float   getTemp()        const { return _multiplusTemp; }
    byte    getLEDon()       const { return _masterMultiLED_LEDon; }
    byte    getLEDblink()    const { return _masterMultiLED_LEDblink; }
    byte    getLEDstatus()   const { return _masterMultiLED_Status; }
    byte    getChargerStatus()  const { return _multiplusStatus80; }
    bool    dcLevelAllowsInverting() const { return _multiplusDcLevelAllowsInverting; }
    float   getMinInputCurrentLimit() const { return _masterMultiLED_MinimumInputCurrentLimit; }
    float   getMaxInputCurrentLimit() const { return _masterMultiLED_MaximumInputCurrentLimit; }
    float   getActInputCurrentLimit() const { return _masterMultiLED_ActualInputCurrentLimit; }
    byte    getSwitchRegister() const { return _masterMultiLED_SwitchRegister; }

    // ----------------------------------------------------------------
    // Status
    // ----------------------------------------------------------------
    bool     hasNewData()  const { return _gotMP2data; }
    void     clearNewData()      { _gotMP2data = false; }
    bool     isAcked()     const { return _acked; }
    void     clearAcked()        { _acked = false; }
    bool     hasNoSync()   const { return _nosync; }
    uint32_t getChecksumFaults() const { return _chksmfault; }

private:
    // The tight-loop task (static entry point + instance method)
    static void _taskEntry(void *param);
    void        _run();

    // RS485 RX processing (same as original multiplusCommandHandling)
    void _processRx();

    // Send a command via RS485
    void _sendCommand(const VEBusCmd &cmd);

    // Frame helpers
    int  _prepareESSCommand(char *out, int16_t power, byte frameNr);
    int  _prepareReadRAM   (char *out, byte frameNr);
    int  _prepareOnOff     (char *out, byte frameNr, bool wakeup);
    int  _replaceFAtoFF    (char *out, const char *in, int len);
    int  _destuffFAtoFF    (char *out, const char *in, int len);
    int  _appendChecksum   (char *buf, int len);
    bool _verifyChecksum   (const char *buf, int len);
    void _decodeFrame      (const char *frame, int len);

    // Pin config (stored for deferred init inside task)
    int _rxPin, _txPin, _dePin;

    // FreeRTOS
    TaskHandle_t  _taskHandle;
    QueueHandle_t _cmdQueue;

    // Frame assembly buffers
    char _frbuf1[128];
    char _frbuf2[128];
    char _txbuf1[32];
    char _txbuf2[32];

    byte _frp;
    byte _frlen;
    byte _frameNr;

    // Protocol state
    volatile bool          _syncrxed;
    volatile bool          _nosync;
    volatile bool          _gotMP2data;
    volatile bool          _acked;
    volatile unsigned long _synctime;
    volatile uint32_t      _chksmfault;

    // Decoded device data (volatile for cross-core reads)
    volatile byte    _masterMultiLED_LEDon;
    volatile byte    _masterMultiLED_LEDblink;
    volatile byte    _masterMultiLED_Status;
    volatile byte    _masterMultiLED_AcInputConfiguration;
    volatile float   _masterMultiLED_MinimumInputCurrentLimit;
    volatile float   _masterMultiLED_MaximumInputCurrentLimit;
    volatile float   _masterMultiLED_ActualInputCurrentLimit;
    volatile byte    _masterMultiLED_SwitchRegister;
    volatile float   _multiplusTemp;
    volatile float   _multiplusDcCurrent;
    volatile byte    _multiplusStatus80;
    volatile bool    _multiplusDcLevelAllowsInverting;
    volatile float   _BatVolt;
    volatile int16_t _ACPower;
};
