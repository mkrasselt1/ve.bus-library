#pragma once

#include <Arduino.h>
#include "driver/uart.h"

// -------------------------------------------------------------------
// LED bitmask constants (returned by getLEDon() / getLEDblink())
// Bits matching masterMultiLED_LEDon from the MK3 protocol
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
// VEBus
//
// Minimal driver for the Victron Multiplus VE.Bus RS485 interface.
// Wraps the MK3 framing protocol:  receive/decode incoming frames
// and assemble/send ESS power, sleep/wake, and RAM-read commands.
//
// Typical usage (single-core, no RTOS):
//
//   VEBus vebus;
//
//   void setup() { vebus.begin(RX_PIN, TX_PIN, DE_PIN); }
//
//   void loop() {
//     vebus.handle();                           // process serial RX
//
//     if (vebus.isSynced() &&
//         millis() >= vebus.getSyncTime() + 8)  // 8 ms tx slot
//     {
//       vebus.sendESSPower(essWatts);           // positive=invert, negative=charge
//     }
//
//     if (vebus.hasNewData()) {
//       vebus.clearNewData();
//       Serial.printf("Bat: %.2f V  AC: %d W\n",
//                     vebus.getBatVolt(), vebus.getACPower());
//     }
//   }
// -------------------------------------------------------------------
class VEBus {
public:
    VEBus();

    // Initialize the RS485 UART.
    // Default baud rate matches the Victron Multiplus (256000).
    void begin(int rxPin, int txPin, int dePin, uint32_t baud = 256000);

    // Process all bytes currently available on the serial port.
    // Must be called frequently (every few ms) from loop() or a task.
    // Updates the sync flag, decoded data fields, and no-sync detection.
    void handle();

    // ----------------------------------------------------------------
    // TX commands — only call after isSynced() is true and the tx
    // slot delay has elapsed (see getSyncTime() + txDelay).
    // ----------------------------------------------------------------

    // Write ESS power setpoint — controls power exchange with the AC-IN (grid) side.
    //   watts > 0 : invert — push watts FROM battery TOWARD grid/AC-IN
    //               (reduces grid import or feeds back into the grid)
    //   watts < 0 : charge — pull watts FROM grid INTO battery
    //   watts = 0 : standby — grid pass-through only, battery idle
    //
    // Note: AC-OUT loads are served by grid pass-through + inverter combined.
    // The setpoint does NOT directly equal the AC-OUT power delivered to loads.
    void sendESSPower(int16_t watts);

    // Request battery voltage (RAM id 0x04) and AC power (RAM id 0x0E).
    // The response arrives asynchronously; poll hasNewData().
    void sendReadRAM();

    // Put the Multiplus into sleep mode.
    void sendSleep();

    // Wake the Multiplus from sleep mode.
    void sendWakeup();

    // ----------------------------------------------------------------
    // Sync / protocol status
    // ----------------------------------------------------------------

    // True for one handle() pass after a sync frame is fully received.
    // Cleared by the next incoming byte (i.e. the start of the next frame).
    bool isSynced()   const { return _syncrxed; }

    // Manually clear the sync flag after you have acted on it.
    void clearSync()        { _syncrxed = false; }

    // Timestamp (millis()) of the last sync frame reception.
    // Use this to implement the required TX slot delay, e.g.:
    //   if (isSynced() && millis() >= getSyncTime() + 8) { sendESSPower(...); }
    unsigned long getSyncTime() const { return _synctime; }

    // True when no sync frame has been seen for more than 1 second.
    bool hasNoSync()  const { return _nosync; }

    // Frame counter from the last sync frame (0x00–0x7F).
    // Your command must use (frameNr + 1) & 0x7F — the library handles
    // this internally in all send*() methods.
    byte getFrameNr() const { return _frameNr; }

    // ----------------------------------------------------------------
    // Data availability
    // ----------------------------------------------------------------

    // True when a RAM-read response (battery voltage + AC power) has
    // arrived since the last clearNewData() call.
    bool hasNewData()  const { return _gotMP2data; }
    void clearNewData()      { _gotMP2data = false; }

    // True when the last sendESSPower() was acknowledged by the device.
    bool isAcked()     const { return _acked; }
    void clearAcked()        { _acked = false; }

    // Running count of frames that failed checksum verification.
    uint32_t getChecksumFaults() const { return _chksmfault; }

    // ----------------------------------------------------------------
    // Decoded Multiplus data
    // ----------------------------------------------------------------

    // Battery voltage [V] — from RAM read response
    float   getBatVolt()     const { return _BatVolt; }

    // AC power [W] — from RAM read response (positive = inverting)
    int16_t getACPower()     const { return _ACPower; }

    // DC battery current [A] — from 0x80 charger/inverter frame
    float   getDCCurrent()   const { return _multiplusDcCurrent; }

    // Multiplus temperature [°C] — from 0x80 frame (when available)
    float   getTemp()        const { return _multiplusTemp; }

    // LED bitmask — use VEBUS_LED_* constants
    byte    getLEDon()       const { return _masterMultiLED_LEDon; }
    byte    getLEDblink()    const { return _masterMultiLED_LEDblink; }

    // Device status byte from the 0x41 LED/mode frame (0 = OK, 2 = bat low)
    byte    getLEDstatus()   const { return _masterMultiLED_Status; }

    // Status byte from the 0x80 charger/inverter frame (0 = OK, 2 = bat low)
    byte    getChargerStatus()  const { return _multiplusStatus80; }

    // True when the DC bus level permits inverting
    bool    dcLevelAllowsInverting() const { return _multiplusDcLevelAllowsInverting; }

    // AC input current limits [A] — from 0x41 LED/mode frame
    float   getMinInputCurrentLimit() const { return _masterMultiLED_MinimumInputCurrentLimit; }
    float   getMaxInputCurrentLimit() const { return _masterMultiLED_MaximumInputCurrentLimit; }
    float   getActInputCurrentLimit() const { return _masterMultiLED_ActualInputCurrentLimit; }

    // Switch register byte — from 0x41 LED/mode frame
    byte    getSwitchRegister() const { return _masterMultiLED_SwitchRegister; }

private:
    // Frame assembly buffers
    char _frbuf1[128]; // raw bytes as received
    char _frbuf2[128]; // after FA-stuffing removal
    char _txbuf1[32];  // command before byte-stuffing & checksum
    char _txbuf2[32];  // final transmit buffer

    byte _frp;         // write pointer into _frbuf1
    byte _frlen;       // length of last decoded frame
    byte _frameNr;     // frame counter from last sync

    // Protocol state
    bool          _syncrxed;
    bool          _gotMP2data;
    bool          _acked;
    bool          _nosync;
    unsigned long _synctime;
    uint32_t      _chksmfault;

    // Decoded device data
    byte    _masterMultiLED_LEDon;
    byte    _masterMultiLED_LEDblink;
    byte    _masterMultiLED_Status;
    byte    _masterMultiLED_AcInputConfiguration;
    float   _masterMultiLED_MinimumInputCurrentLimit;
    float   _masterMultiLED_MaximumInputCurrentLimit;
    float   _masterMultiLED_ActualInputCurrentLimit;
    byte    _masterMultiLED_SwitchRegister;
    float   _multiplusTemp;
    float   _multiplusDcCurrent;
    byte    _multiplusStatus80;
    bool    _multiplusDcLevelAllowsInverting;
    float   _BatVolt;
    int16_t _ACPower;

    // Internal frame helpers
    int  _prepareESSCommand(char *out, int16_t power, byte frameNr);
    int  _prepareReadRAM   (char *out, byte frameNr);
    int  _prepareOnOff     (char *out, byte frameNr, bool wakeup);
    int  _replaceFAtoFF    (char *out, const char *in, int len);
    int  _destuffFAtoFF    (char *out, const char *in, int len);
    int  _appendChecksum   (char *buf, int len);
    bool _verifyChecksum   (const char *buf, int len);
    void _sendFrame        (char *raw, int rawLen);
    void _decodeFrame      (const char *frame, int len);
};
